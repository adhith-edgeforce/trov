#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

class GPIOPin32Node(Node):
    def __init__(self):
        super().__init__('gpio_pin32_node')

        self.PIN = 32  # Physical board pin 32

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)
        self.get_logger().info("Pin 32 initialized LOW")

        self.subscription = self.create_subscription(
            Bool,
            'gpio_pin32_control',
            self.gpio_callback,
            10
        )
        self.get_logger().info("Subscribed to 'gpio_pin32_control'")

    def gpio_callback(self, msg: Bool):
        if msg.data:
            GPIO.output(self.PIN, GPIO.HIGH)
            self.get_logger().info("Pin 32 → HIGH")
        else:
            GPIO.output(self.PIN, GPIO.LOW)
            self.get_logger().info("Pin 32 → LOW")

    def destroy_node(self):
        GPIO.output(self.PIN, GPIO.LOW)
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOPin32Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()