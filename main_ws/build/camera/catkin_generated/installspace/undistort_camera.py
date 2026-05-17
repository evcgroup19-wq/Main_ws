#!/usr/bin/env python2

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CV2Subscriber:
    def __init__(self, node_name, topic):
        self.latest_frame = None
        rospy.Subscriber(topic, CompressedImage, self._callback)

    def _callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

class CV2Publisher:
    def __init__(self, node_name, topic):
        self.publisher = rospy.Publisher(topic, CompressedImage, queue_size=10)

    def pub_frame(self, frame):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        _, encoded = cv2.imencode(".jpg", frame)
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)

def main():
    rospy.init_node('camera_processor', anonymous=True)

    subscriber = CV2Subscriber('camera_processed_publisher', '/camera/video_raw')
    publisher = CV2Publisher('camera_processed_subscriber', '/camera/video_processed')

    camera_settings = np.load('/home/jetbot/Aman_ENV/EVC/Aman_ws/calibrations/mse=0.6576723536152839.npz')
    mtx = camera_settings["mtx"]
    dist = camera_settings["dist"]
    newcameramtx = camera_settings["newcameramtx"]
    roi = camera_settings["roi"]

    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    fontsize = 3
    (_, line_height), _ = cv2.getTextSize("example", fontFace, fontScale, fontsize)

    rate = rospy.Rate(30)
    rospy.loginfo("Starting camera processor...")

    try:
        while not rospy.is_shutdown():
            img = subscriber.latest_frame

            if img is not None:
                rospy.loginfo_once("First frame received!")
                dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

                cv2.putText(img, "raw",
                    (4, line_height + 2), fontFace, fontScale,
                    (255, 255, 255), fontsize, cv2.LINE_AA)

                cv2.putText(dst, "processed",
                    (4, line_height + 2), fontFace, fontScale,
                    (255, 255, 255), fontsize, cv2.LINE_AA)

                #cv2.imshow("Raw vs Undistorted", stacked)
                publisher.pub_frame(dst)

            else:
                rospy.logwarn_throttle(5, "Waiting for frames on /camera/image_raw ...")

            cv2.waitKey(1)
            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()