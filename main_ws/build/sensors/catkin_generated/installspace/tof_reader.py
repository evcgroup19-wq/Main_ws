#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
from sense_lib.tof_driver import VL53L0X
from sense_lib import topics
def main():
    # Initialize ROS node
    rospy.init_node('tof_node', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher(topics.TOF_DATA, Float32, queue_size=10)
    
    rate = rospy.Rate(10)  # 10hz
    
    sensor = VL53L0X()
    
    rospy.loginfo("TOF sensor node started, publishing to /sensor/tof")
    
    try:
        while not rospy.is_shutdown():
            distance = sensor.read_distance()
            
            if distance is not None:
                pub.publish(float(distance))
                #DBG trace
                #rospy.loginfo("Distance: {} mm".format(distance))
            
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("Measurement stopped by User")
    
    finally:
        sensor.close()

if __name__ == '__main__':
    main()