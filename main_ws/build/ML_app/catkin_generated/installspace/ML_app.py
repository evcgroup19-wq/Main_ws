#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import json
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Motion state constants
CLEAR   = "CLEAR"
CAUTION = "CAUTION"
STOP    = "STOP"


class MotionDetector:
    def __init__(self):
        rospy.init_node("motion_detection_node", anonymous=False)
        rospy.loginfo("[MotionDetector] Node starting ...")

        self.min_contour_area  = rospy.get_param("~min_contour_area",  500)
        self.blur_kernel_size  = rospy.get_param("~blur_kernel_size",  21)
        self.dilate_iterations = rospy.get_param("~dilate_iterations", 2)
        self.learning_rate     = rospy.get_param("~learning_rate",     0.01)
        self.alert_cooldown    = rospy.get_param("~alert_cooldown",    2.0)
        history                = rospy.get_param("~history",           200)
        detect_shadows         = rospy.get_param("~detect_shadows",    False)

        self.zone = {
            "x1": rospy.get_param("~zone_x1", -1),
            "y1": rospy.get_param("~zone_y1", -1),
            "x2": rospy.get_param("~zone_x2", -1),
            "y2": rospy.get_param("~zone_y2", -1),
        }

        self.speed_cruise      = rospy.get_param("~speed_cruise",      0.15)
        self.speed_caution     = rospy.get_param("~speed_caution",     0.05)
        self.caution_threshold = rospy.get_param("~caution_threshold", 5000)
        self.stop_threshold    = rospy.get_param("~stop_threshold",    50000)
        self.backup_speed      = rospy.get_param("~backup_speed",     -0.08)
        self.backup_duration   = rospy.get_param("~backup_duration",   1.0)

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=history, varThreshold=16, detectShadows=detect_shadows
        )

        if self.blur_kernel_size % 2 == 0:
            self.blur_kernel_size += 1

        self.bridge          = CvBridge()
        self.last_alert_time = 0.0
        self.backup_until    = 0.0
        self.frame_count     = 0
        self.alert_count     = 0
        self.motion_state    = CLEAR

        self.pub_alert = rospy.Publisher("/motion/alert", String, queue_size=10)
        self.pub_cmd   = rospy.Publisher("/motor/cmd",   Twist,  queue_size=10)

        self.sub = rospy.Subscriber(
            "/camera/video_processed", CompressedImage,
            self.image_callback, queue_size=5, buff_size=2 ** 24,
        )

        rospy.loginfo("[MotionDetector] Ready.")
        rospy.spin()

    def _get_zone(self, h, w):
        x1 = self.zone["x1"] if self.zone["x1"] >= 0 else 0
        y1 = self.zone["y1"] if self.zone["y1"] >= 0 else 0
        x2 = self.zone["x2"] if self.zone["x2"] >= 0 else w
        y2 = self.zone["y2"] if self.zone["y2"] >= 0 else h
        return x1, y1, x2, y2

    def _classify_state(self, total_area):
        if total_area >= self.stop_threshold:
            return STOP
        elif total_area >= self.caution_threshold:
            return CAUTION
        return CLEAR

    def _build_twist(self, state, now):
        twist = Twist()
        
        # IF WE ARE IN THE BACKUP LOCKOUT TIME, FORCE A BACKUP AND IGNORE VISION
        if now < self.backup_until:
            twist.linear.x  = self.backup_speed
            twist.angular.z = 0.0
            return twist  # Exit early! Don't let new frames override this.
            
        # Normal operations if we aren't locked in a backup phase
        if state == CLEAR:
            twist.linear.x  = self.speed_cruise
            twist.angular.z = 0.0
        elif state == CAUTION:
            twist.linear.x  = self.speed_caution
            twist.angular.z = 0.0
        elif state == STOP:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            # Trigger the 1-second backup lockout timer
            self.backup_until = now + self.backup_duration
            
        return twist

    def _build_alert(self, contours, frame_shape, stamp, state, total_area):
        blobs = []
        for c in contours:
            x, y, bw, bh = cv2.boundingRect(c)
            blobs.append({
                "x": int(x), "y": int(y),
                "w": int(bw), "h": int(bh),
                "area": int(cv2.contourArea(c)),
            })
        return json.dumps({
            "alert": "INTRUSION_DETECTED",
            "state": state,
            "total_fg_area": int(total_area),
            "timestamp": stamp.to_sec(),
            "frame": self.frame_count,
            "alert_count": self.alert_count,
            "frame_size": {"width": frame_shape[1], "height": frame_shape[0]},
            "blobs": blobs,
        })

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("[MotionDetector] Decode error: %s", e)
            return

        h, w = frame.shape[:2]
        x1, y1, x2, y2 = self._get_zone(h, w)

        roi = frame[y1:y2, x1:x2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(
            gray, (self.blur_kernel_size, self.blur_kernel_size), 0
        )

        fg_mask = self.bg_subtractor.apply(blurred, learningRate=self.learning_rate)
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=self.dilate_iterations)

        contours, _ = cv2.findContours(
            fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        significant = [
            c for c in contours if cv2.contourArea(c) >= self.min_contour_area
        ]

        total_area = sum(cv2.contourArea(c) for c in significant)

        now = time.time()
        state = self._classify_state(total_area)
        twist = self._build_twist(state, now)

        self.pub_cmd.publish(twist)

        # ---------------- ALERT ---------------- #
        if significant and (now - self.last_alert_time) >= self.alert_cooldown:
            self.alert_count += 1
            self.last_alert_time = now

            self.pub_alert.publish(String(data=self._build_alert(
                significant, frame.shape, msg.header.stamp, state, total_area
            )))

        # ---------------- VISUALIZATION FIX ---------------- #
        display = frame.copy()

        # draw ROI box
        cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 0), 1)

        # draw detections
        for c in significant:
            x, y, bw, bh = cv2.boundingRect(c)
            cv2.rectangle(
                display,
                (x1 + x, y1 + y),
                (x1 + x + bw, y1 + y + bh),
                (0, 0, 255),
                2
            )

        # HUD
        cv2.putText(
            display,
            "STATE: %s | AREA: %d" % (state, total_area),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        #cv2.imshow("Motion Detection", display)
        #cv2.imshow("FG Mask", fg_mask)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        MotionDetector()
    except rospy.ROSInterruptException:
        pass