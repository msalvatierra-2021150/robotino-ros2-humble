#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class PolicyExecutor(Node):
    def __init__(self):
        super().__init__('policy_executor')
        self.get_logger().info('Robotino e-MDB policy executor started.')


def main(args=None):
    rclpy.init(args=args)
    node = PolicyExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()