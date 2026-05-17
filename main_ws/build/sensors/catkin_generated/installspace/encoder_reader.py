#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
# This leverages the setup.py global path resolution we verified earlier
from sense_lib.encoder_driver import WheelEncoderDriver 
from sense_lib import topics
# --- HARDWARE CONFIGURATION ---
GPIO_LEFT_ENCODER = 18   # Motor Encoder 1
GPIO_RIGHT_ENCODER = 19  # Motor Encoder 2

def main():
    # Initialize the ROS Node
    rospy.init_node('encoder_node', anonymous=False)
    
    # Create ROS Publishers for both Left and Right sides
    left_rad_pub = rospy.Publisher(topics.ENCODER_LEFT_RAD, Float32, queue_size=10)
    left_dist_pub = rospy.Publisher(topics.ENCODER_LEFT_DIST, Float32, queue_size=10)
    
    right_rad_pub = rospy.Publisher(topics.ENCODER_RIGHT_RAD, Float32, queue_size=10)
    right_dist_pub = rospy.Publisher(topics.ENCODER_RIGHT_DIST, Float32, queue_size=10)
    
    # Set execution frequency (20 Hz is ideal for differential wheel tracking)
    rate = rospy.Rate(20)
    
    # Initialize both hardware drivers
    try:
        left_encoder = WheelEncoderDriver(gpio_pin=GPIO_LEFT_ENCODER)
        right_encoder = WheelEncoderDriver(gpio_pin=GPIO_RIGHT_ENCODER)
        rospy.loginfo("Successfully initialized Left Encoder (Pin {}) and Right Encoder (Pin {})".format(GPIO_LEFT_ENCODER, GPIO_RIGHT_ENCODER))
    except Exception as e:
        rospy.logerr("Critical: Failed to initialize hardware drivers: {}".format(e))
        return

    rospy.loginfo("Dual Encoder reader node online and publishing...")
    
    try:
        while not rospy.is_shutdown():
            # Gather readings safely handled by your driver classes
            left_rad = left_encoder.get_radians()
            left_dist = left_encoder.get_distance_meters()
            
            right_rad = right_encoder.get_radians()
            right_dist = right_encoder.get_distance_meters()
            
            # Publish Left Wheel Telemetry
            left_rad_pub.publish(Float32(left_rad))
            left_dist_pub.publish(Float32(left_dist))
            
            # Publish Right Wheel Telemetry
            right_rad_pub.publish(Float32(right_rad))
            right_dist_pub.publish(Float32(right_dist))
            
            # Optional: ROS debug logging (Uncomment line below to monitor via command line)
            # rospy.loginfo("L_Dist: {:.3f}m | R_Dist: {:.3f}m".format(left_dist, right_dist))
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Encoder node received shutdown interrupt.")
        
    finally:
        # Prevent pin leakage/dangling interrupts by cleanly detaching handlers
        rospy.loginfo("Releasing encoder GPIO event detection interrupts...")
        left_encoder.shutdown()
        right_encoder.shutdown()

if __name__ == '__main__':
    main()