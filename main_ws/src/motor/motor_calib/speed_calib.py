#!/usr/bin/env python2

from motor_lib.motorDriver import *
from time import sleep
# Fixed: Use absolute import for catkin-installed packages
from sense_lib import topics 
import rospy
from std_msgs.msg import Float32

class motor_calib:
    def __init__(self):
        self.motor = DaguWheelsDriver()
        self.start = 0
        self.start_rads = 0.0
        self.prev_rads = 0.0
        
        # Fixed: Typo in rospy.loginfo
        rospy.init_node("calib_node", anonymous=False)
        rospy.loginfo("Starting calib")

        # Register the shutdown callback to turn off motors on exit
        rospy.on_shutdown(self.cleanup)

        self.sub = rospy.Subscriber(
            topics.ENCODER_RIGHT_RAD, Float32,
            self.LEC, queue_size=10, buff_size=10
        )
        rospy.loginfo("ready")
        rospy.spin()

    def LEC(self, data):
        # Fixed: Added missing colon
        if self.start == 0:
            self.start = 1  # Fixed: Changed '==' to '=' for assignment
            self.start_rads = data.data # Note: ROS messages require '.data' to extract the raw value
            self.prev_rads = data.data
            self.motor.set_wheels_speed(1, 1)         
        # Fixed: Added missing colon
        else:
            if self.prev_rads != self.start_rads:
                # Fixed: Accessing variables via 'self' so they persist across callbacks
                print("rad diff {}".format(data.data - self.prev_rads))
            
            # Update prev_rads for the next callback iteration
            self.prev_rads = data.data

    def cleanup(self):
        """
        This function automatically runs when the node is killed (Ctrl+C).
        It commands the motors to stop safely.
        """
        rospy.loginfo("Shutting down node. Stopping motors safely...")
        try:
            # Assuming set_wheels_speed(0, 0) stops your specific motor driver
            self.motor.set_wheels_speed(0, 0)
        except Exception as e:
            rospy.logerr("Failed to stop motors during shutdown: {}".format(e))

if __name__ == "__main__":
    try:
        motor_calib()
    except rospy.ROSInterruptException:
        pass