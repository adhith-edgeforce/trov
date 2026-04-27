#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class NavSatRepublisher(Node):
    def __init__(self):
        super().__init__('navsat_republisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.callback,
            qos_profile
        )

        self.pub = self.create_publisher(
            NavSatFix,
            '/navsat/fix',
            10
        )

        self.get_logger().info("NavSat republisher started")

    def callback(self, msg):
        msg.header.frame_id = 'gps_link'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NavSatRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()