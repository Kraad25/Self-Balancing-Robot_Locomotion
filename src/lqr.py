#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from control import lqr
from std_msgs.msg import Float32, String
from enum import Enum, auto


# States
class States(Enum):
    STATIC = auto()
    TRANSLATION = auto()
    ROTATIONAL = auto()
    INITIATE_LEAN = auto()
    RETURN_TO_STATIC = auto()

# System and control matrices
A = np.array([[0, 1], [5, 0]])
B = np.array([[0], [9.0]])

MAX_TILT_ANGLE = 45.0  # Stop correction beyond 45 degrees
TURN_THRESHOLD = 10.0  # While rotating, if tilts exceeds threshold, prioritize balancing
LEAN_TILT = 20.0 # Tilt allowed for leaning during translation motion
TILT_ERROR_THRESHOLD = 1.0
ROTATIONAL_VELOCITY = 2.0

class LQR(Node):
    def __init__(self):    
        super().__init__("lqr_controller")
        self.get_logger().info("LQR Controller Initiliazed")

        # Publisher and Subscription Definition
        self.lqr_input = self.create_subscription(Imu, '/lqr_input', self.lqrCallback, 10)
        self.motor_cmds_pub = self.create_publisher(Twist, '/motor_cmds', 10)
        self.movement_sub = self.create_subscription(String, '/user_input', self.movementCallback, 10)

        # LQR
        self.Q = np.array([[10, 0], [0, 30]])
        self.R = 2.0
        self.K, self.S, self.E = lqr(A, B, self.Q, self.R)

        # State Machine
        self.state = States.STATIC
        self.desired_movement = 'Stop'
        self.target_tilt = 0.0  # Target tilt for leaning

    def movementCallback(self, msg: String):
        self.desired_movement = msg.data
        self.get_logger().info(f"Movement command: {self.desired_movement}")

        if self.state == States.TRANSLATION and self.desired_movement in ['Forward', 'Backward']:
            self.state = States.RETURN_TO_STATIC
            self.get_logger().info("Switching directions. Returning to STATIC first.")

    def lqrCallback(self, data: Imu):
        # Process IMU data and compute motor commands using LQR
        tilt_angle = data.orientation.y * 180 / np.pi  # Convert pitch (tilt) to degrees
        tilt_rate = data.angular_velocity.y 

        if abs(tilt_angle) > MAX_TILT_ANGLE:
            self.motor_cmds_pub.publish(Twist())
            self.state = States.STATIC
            self.get_logger().warn("MAX TILT REACHED! Stopping motors.")


        np_x = np.array([[tilt_angle], [tilt_rate]])
        feedback = self.K.dot(np_x)[0, 0] # Calculate feedback gain for tilt

        
        ##########################################   State Machine   ####################################################
        motor_vel = Twist()
        if self.state == States.STATIC:
            self.handle_static(motor_vel, feedback)

        elif self.state == States.INITIATE_LEAN:
            self.handle_initiate_lean(motor_vel, tilt_angle, tilt_rate)

        elif self.state == States.TRANSLATION:
            self.handle_translation_motion(motor_vel, tilt_angle, tilt_rate)

        elif self.state == States.ROTATIONAL:
            self.handle_rotational_motion(motor_vel, tilt_angle, feedback)

        elif self.state == States.RETURN_TO_STATIC:
            self.handle_return_to_static(motor_vel, tilt_angle, feedback)
        
        self.motor_cmds_pub.publish(motor_vel)
        
        ################################################################################################################

    
    def handle_static(self, motor_vel: Twist, feedback: float):
                    # Static State #
        motor_vel.linear.x = motor_vel.linear.y = feedback

        if self.desired_movement in ['Left', 'Right']:
            self.state = States.ROTATIONAL
            self.get_logger().info("Transitioning to ROTATIONAL state.")
        elif self.desired_movement in ['Forward', 'Backward']:
            self.state = States.INITIATE_LEAN
            self.target_tilt = LEAN_TILT if self.desired_movement == 'Forward' else -LEAN_TILT
            self.get_logger().info("Initiating tilt for TRANSLATION.")

    def handle_initiate_lean(self, motor_vel: Twist, tilt_angle: float, tilt_rate: float):
                    # Initiate Lean State #
        tilt_error = tilt_angle - self.target_tilt
        np_x = np.array([[tilt_error], [tilt_rate]])
        feedback = self.K.dot(np_x)[0, 0]

        motor_vel.linear.x = feedback
        motor_vel.linear.y = feedback

        if abs(tilt_error) < TILT_ERROR_THRESHOLD:  # Close to desired tilt, transition to TRANSLATION
            self.state = States.TRANSLATION
            self.get_logger().info("Transitioning to TRANSLATION state.")

    def handle_translation_motion(self, motor_vel: Twist, tilt_angle: float, tilt_rate: float):
                    # Translation Motion #
        tilt_error = tilt_angle - self.target_tilt
        np_x = np.array([[tilt_error], [tilt_rate]])
        feedback = self.K.dot(np_x)[0, 0]

        net_velocity = (0.5 * feedback)
        motor_vel.linear.x = net_velocity
        motor_vel.linear.y = net_velocity

        if self.desired_movement not in ['Forward', 'Backward']:
            self.state = States.RETURN_TO_STATIC
            self.get_logger().info("Returning to STATIC state.")

    def handle_rotational_motion(self, motor_vel: Twist, tilt_angle: float, feedback: float):
                    # Rotational Motion # 
        if self.desired_movement == 'Left':
            motor_vel.linear.x = -ROTATIONAL_VELOCITY # Slow left wheel
            motor_vel.linear.y = ROTATIONAL_VELOCITY # Speed up right wheel
        elif self.desired_movement == 'Right':
            motor_vel.linear.x = ROTATIONAL_VELOCITY # Speed up left wheel
            motor_vel.linear.y = -ROTATIONAL_VELOCITY # Slow right wheel

        # Return to STATIC if the robot tilt exceeds the threshold during motion
        if abs(tilt_angle) > TURN_THRESHOLD:
            self.state = States.STATIC

    def handle_return_to_static(self, motor_vel: Twist, tilt_angle: float, feedback: float):
                    # Return to Static state #
        motor_vel.linear.x = feedback
        motor_vel.linear.y = feedback

        if abs(tilt_angle) < 1.0:  # If close to upright, transition to STATIC
            self.state = States.STATIC
            self.get_logger().info("Returned to STATIC state.")

    

if __name__ == '__main__':
    rclpy.init()
    lqr_values = LQR()
    rclpy.spin(lqr_values)
    lqr_values.destroy_node()
    rclpy.shutdown()
