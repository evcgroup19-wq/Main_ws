#!/usr/bin/env python2

import rospy
import time
from sensor_msgs.msg import Imu
# Custom topic definitions from your shared package
from sense_lib import topics 
# Assuming you want to publish to a motor control topic, or import the driver directly
from motor_lib.motorDriver import DaguWheelsDriver

class IMUStraightLinePID:
    def __init__(self):
        rospy.init_node("imu_pid_controller", anonymous=False)
        rospy.loginfo("Initializing IMU Heading Hold PID Controller...")
        
        # Initialize your physical motor driver
        self.motor = DaguWheelsDriver()
        
        # --- PID TUNING PARAMETERS ---
        # Proportional: Aggressively responds to the current error
        self.Kp = 1.9  
        # Integral: Gradually grows over time if drift persists (fixes battery drain)
        self.Ki = 0.38  
        # Derivative: Damps oscillations so the bot doesn't wag its tail
        self.Kd = 0.20  
        
        # --- PID STATE VARIABLES ---
        self.target_heading = 0.0      # We want to maintain our initial heading (0 radians deviation)
        self.current_heading = 0.0     # Calculated via gyro integration
        self.integral_error = 0.0      # Accumulated error over time
        self.last_error = 0.0          # Previous loop error for derivative calculation
        self.last_time = rospy.get_time()
        
        # --- BASE MOTION SPEED ---
        self.base_speed = 1         # Target driving speed (0.0 to 1.0)
        self.is_moving = True          # Control flag
        
        # Subscribe to your IMU data topic
        self.imu_sub = rospy.Subscriber(
            topics.IMU_DATA, Imu, self.imu_callback, queue_size=10
        )
        
        # Register safety shutdown hook to kill motors if node closes
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("PID Node Ready. Driving Straight...")

    def imu_callback(self, msg):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        
        # Guard against zero or negative time deltas
        if dt <= 0.0:
            return
            
        # 1. INTEGRATION STEP: Gyro Z outputs rad/s. Multiply by time (dt) to get change in angle.
        # Your node cleanly outputs standard ROS radians/sec for angular velocity.
        yaw_rate = msg.angular_velocity.z
        self.current_heading += yaw_rate * dt
        self.last_time = current_time
        
        # 2. CALCULATE ERROR
        # Error is how far we have strayed from 0.0. 
        # If drifting left, yaw_rate is positive -> heading becomes positive -> error becomes negative.
        error = self.target_heading - self.current_heading
        
        # 3. PID MATHEMATICAL CALCULATIONS
        # Proportional Term
        p_term = self.Kp * error
        
        # Integral Term (with anti-windup clamping to prevent runaway power saturation)
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, 1.0), -1.0) 
        i_term = self.Ki * self.integral_error
        
        # Derivative Term
        derivative = (error - self.last_error) / dt
        d_term = self.Kd * derivative
        self.last_error = error
        
        # Total Correction Output
        correction = p_term + i_term + d_term
        
        # 4. APPLY CORRECTION TO MOTORS
        if self.is_moving:
            # If correction is positive, robot needs to turn left (Speed up Right, Slow down Left)
            # If correction is negative, robot needs to turn right (Speed up Left, Slow down Right)
            left_speed = self.base_speed - correction
            right_speed = self.base_speed + correction
            
            # Clamp final values to valid hardware limits [0.0, 1.0]
            left_speed = max(min(left_speed, 1.0), 0.0)
            right_speed = max(min(right_speed, 1.0), 0.0)
            
            # Send dynamic adjustments straight to the wheels
            self.motor.set_wheels_speed(left_speed, right_speed)
            
            # Debugging readout to monitor corrections in real time
            rospy.logdebug("Heading: {:.3f} rad | Error: {:.3f} | Correction: {:.3f} | L: {:.2f} R: {:.2f}".format(
                self.current_heading, error, correction, left_speed, right_speed
            ))

    def cleanup(self):
        rospy.loginfo("Stopping motors safely via PID Cleanup Hook...")
        self.is_moving = False
        self.motor.set_wheels_speed(0, 0)

if __name__ == '__main__':
    try:
        controller = IMUStraightLinePID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass