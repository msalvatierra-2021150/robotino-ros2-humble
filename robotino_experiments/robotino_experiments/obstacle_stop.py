import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud


class ObstacleStop(Node):
    def __init__(self):
        super().__init__("obstacle_stop")

        self.declare_parameter("forward_speed", 0.05)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("front_width", 0.45)

        self.forward_speed = self.get_parameter("forward_speed").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.front_width = self.get_parameter("front_width").value

        self.obstacle_detected = False

        self.sub = self.create_subscription(
            PointCloud,
            "/rto3/distance_sensors",
            self.sensor_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/rto3/cmd_vel",
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Obstacle stop node started.")

    def sensor_callback(self, msg):
        obstacle = False

        for p in msg.points:
            # Only care about points in front of the robot.
            # base_link convention: +x is forward, +y is left.
            if p.x <= 0.0:
                continue

            # Ignore points far to the side.
            if abs(p.y) > self.front_width:
                continue

            distance = math.sqrt(p.x ** 2 + p.y ** 2)

            if distance < self.stop_distance:
                obstacle = True
                break

        self.obstacle_detected = obstacle

    def control_loop(self):
        cmd = Twist()

        if self.obstacle_detected:
            cmd.linear.x = 0.0
            self.get_logger().warn("Obstacle detected. Stopping.", throttle_duration_sec=1.0)
        else:
            cmd.linear.x = self.forward_speed

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()