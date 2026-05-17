#!/usr/bin/env python2

import rospy
import math
from sensor_msgs.msg import Imu
from sense_lib.imu_driver import mpu6050
from sense_lib import topics

class IMUReaderNode():
    def __init__(self, node_name, freq_divider, ang_vel_offset, accel_offset, i2c_bus, device_address):
        self.initialized = False
        rospy.loginfo("Initializing IMU reader node...")
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        
        self.DMP_freq = 40
        self.g = 9.80665 # Earth gravity constant
        self._freq_divider = freq_divider
        
        # Offsets stored directly in physical units (m/s^2 and degrees/s)
        self._gyro_offset = ang_vel_offset
        self._accel_offset = accel_offset
        self._bus = i2c_bus
        self._address = device_address
        self._offset_init = False
        
        # Initialize hardware using your driver
        self._sensor = self._find_sensor()
        if not self._sensor:
            rospy.logerr("No MPU6050 device found on bus {} at address 0x{:02X}!".format(self._bus, self._address))
            exit(1)
        
        # ROS Publisher using centralized topic registry
        self.pub = rospy.Publisher(topics.IMU_DATA, Imu, queue_size=10)
        
        # If no default offsets are provided, dynamically calibrate on startup
        if self._accel_offset == [0, 0, 0] and not self._offset_init:
            self.zero_sensor()
            self._offset_init = True

        # Setup loop execution timer
        if self._freq_divider > 0:
            period = float(self._freq_divider) / float(self.DMP_freq)
            self.timer = rospy.Timer(rospy.Duration.from_sec(period), self.publish_data)
        else:
            self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_data)
            
        rospy.loginfo("IMU reader node initialized successfully!")

    def _find_sensor(self):
        try:
            sensor = mpu6050(self._address, bus=self._bus)
            return sensor
        except Exception as e:
            rospy.logwarn("Failed to attach to sensor driver: {}".format(e))
            return None

    def publish_data(self, event):
        msg = Imu()
        try:
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_link"

            # 1. Fetch data (Driver automatically yields m/s^2 and deg/s)
            acc_data = self._sensor.get_accel_data()
            gyro_data = self._sensor.get_gyro_data()

            # 2. Orientation Data (Set covariance to -1 since MPU6050 lacks internal fused orientation)
            msg.orientation.w = 1.0  
            msg.orientation_covariance[0] = -1

            # 3. Angular Velocity: ROS requires RADIANS/sec. Your driver outputs DEGREES/sec.
            # Conversion: degrees * (pi / 180)
            deg_to_rad = math.pi / 180.0
            msg.angular_velocity.x = (gyro_data['x'] - self._gyro_offset[0]) * deg_to_rad
            msg.angular_velocity.y = (gyro_data['y'] - self._gyro_offset[1]) * deg_to_rad
            msg.angular_velocity.z = (gyro_data['z'] - self._gyro_offset[2]) * deg_to_rad

            # 4. Linear Acceleration: Driver is already in m/s^2. Just apply offsets.
            msg.linear_acceleration.x = acc_data['x'] - self._accel_offset[0]
            msg.linear_acceleration.y = acc_data['y'] - self._accel_offset[1]
            msg.linear_acceleration.z = acc_data['z'] - self._accel_offset[2]

            # 5. Emit message
            self.pub.publish(msg)

        except Exception as IMUCommLoss:
            rospy.logwarn("IMU Communication Issue: {}".format(IMUCommLoss))

    def zero_sensor(self):
        rospy.loginfo("Calibrating IMU offsets. Ensure the robot is flat and stationary...")
        rospy.sleep(0.5) # Allow readings to settle
        
        acc_data = self._sensor.get_accel_data()
        gyro_data = self._sensor.get_gyro_data()

        # Deduct natural 1g (~9.81 m/s^2) gravity vector from the Z-axis offset calculation
        # This keeps the gravity reading intact when the robot is sitting still
        self._accel_offset = [
            acc_data['x'],
            acc_data['y'],
            acc_data['z'] - self.g  
        ]

        # Gyro reads absolute 0 when perfectly still
        self._gyro_offset = [gyro_data['x'], gyro_data['y'], gyro_data['z']]
        
        rospy.loginfo("Calibration finished.")
        rospy.loginfo("Acc Offset: X:{:.2f}, Y:{:.2f}, Z:{:.2f} m/s^2".format(*self._accel_offset))
        rospy.loginfo("Gyro Offset: X:{:.2f}, Y:{:.2f}, Z:{:.2f} deg/s".format(*self._gyro_offset))

if __name__ == '__main__':
    node = IMUReaderNode(node_name="imu_node", freq_divider=8, ang_vel_offset=[0, 0, 0], 
                         accel_offset=[0, 0, 0], i2c_bus=1, device_address=0x68)
    rospy.spin()