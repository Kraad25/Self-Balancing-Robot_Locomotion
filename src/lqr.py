#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from control import lqr
from std_msgs.msg import Float32, String


# System and control matrices
A = np.array([[0, 1], [5, 0]])
B = np.array([[0], [9.0]])

MAX_TILT_ANGLE = 45.0  # Stop correction beyond 45 degrees
TILT_THRESHOLD = 10.0  # While in motion if it tilts beyond threshold prioritize balancing

class LQR(Node):
    def __init__(self):    
        super().__init__("lqr_values")
        self.get_logger().info("LQR is Computing Values")

        # Publisher and Subscription Definition
        self.lqr_inp = self.create_subscription(Imu, '/lqr_input', self.lqrCallback, 10)
        self.lqr_output = self.create_publisher(Twist, '/motor_cmds', 10)
        self.movement = self.create_subscription(String, '/isMoving', self.movementCallback, 10)

        # LQR
        self.Q = np.array([[20, 0], [0, 50]])
        self.R = 2.0
        self.K, self.S, self.E = lqr(A, B, self.Q, self.R)

        # State Machine
        self.state = 'STATIC'
        self.desired_movement = 'Stop'

    def movementCallback(self, msg: String):
        self.desired_movement = msg.data
        self.get_logger().info(f"Movement command: {self.desired_movement}")

    def lqrCallback(self, data: Imu):
        # Read tilt and angular velocity directly from IMU message
        tilt_angle = data.orientation.y * 180 / 3.1416  # Convert pitch (tilt) to degrees
        tilt_rate = data.angular_velocity.y 

        np_x = np.array([[tilt_angle], [tilt_rate]])
        feedback = - self.K.dot(np_x)[0, 0] # Calculate feedback gain for tilt
        motor_vel = Twist()

        if self.state == 'STATIC':
            motor_vel.linear.x = -float(feedback)  # Left wheel speed
            motor_vel.linear.y = -float(feedback)   # Right wheel speed

            if self.desired_movement in ['Left', 'Right', 'Forward', 'Backward']:
                self.state = 'MOTION'
                self.get_logger().info("Transitioning to MOVING State")

        elif self.state == 'MOTION':
            
            if self.desired_movement == 'Left':
                motor_vel.linear.x = -2.0 # Slow left wheel
                motor_vel.linear.x = 2.0 # Speed up right wheel
            elif self.desired_movement == 'Right':
                motor_vel.linear.x = 2.0 # Speed up left wheel
                motor_vel.linear.x = -2.0 # Slow right wheel
            elif self.desired_movement == 'Forward':
                pass
            elif self.desired_movement == 'Backward':
                pass
            else:
                self.state = 'STATIC'

            # Return to STATIC if the robot tilt exceeds the threshold during motion
            if abs(tilt_angle) > TILT_THRESHOLD:
                self.state = "STATIC"
                self.get_logger().info("Tilt exceeded threshold! Returning to STATIC.")


        if abs(tilt_angle) > MAX_TILT_ANGLE:
            motor_vel.linear.x = 0.0
            motor_vel.linear.y = 0.0
            self.state = "STATIC"
            self.get_logger().info("MAX TILT REACHED! Stopping motors.")

        self.lqr_output.publish(motor_vel)


if __name__ == '__main__':
    rclpy.init()
    lqr_values = LQR()
    rclpy.spin(lqr_values)
    lqr_values.destroy_node()
    rclpy.shutdown()
