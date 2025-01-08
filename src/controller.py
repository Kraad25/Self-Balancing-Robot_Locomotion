#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import termios
import tty
import sys
import select

class WASDController(Node):
    def __init__(self):
        super().__init__('wasd_controller')
        self.get_logger().info("Initialized, Ready to be Controlled")
        self.publisher = self.create_publisher(String, '/isMoving', 10)

        self.timer = self.create_timer(0.1, self.check_key)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def check_key(self):
        key = self.get_key()
        msg = String()

        if key == 'w':
            msg.data = 'Forward'  # Move forward
        elif key == 's':
            msg.data = 'Backward' # Move backward
        elif key == 'a':
            msg.data = 'Left'   # Turn Left
        elif key == 'd':
            msg.data = 'Right'  # Turn Right
        elif key == 'x':
            msg.data = 'Stop' # Reset

        # Publish the desired tilt
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Desired Tilt: {msg.data}')

        if key == '':  # CTRL-C to exit
            self.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    wasd_controller = WASDController()
    rclpy.spin(wasd_controller)
    wasd_controller.destroy_node()
    rclpy.shutdown()
