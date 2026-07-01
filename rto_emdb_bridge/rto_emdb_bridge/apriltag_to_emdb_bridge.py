import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32, Float32
from apriltag_msgs.msg import AprilTagDetectionArray


class AprilTagToEMDBBridge(Node):
    def __init__(self):
        super().__init__('apriltag_to_emdb_bridge')

        # Parameters
        self.declare_parameter('target_tag_id', 3)
        self.declare_parameter('image_width', 512.0)
        self.declare_parameter('image_height', 384.0)
        self.declare_parameter('confidence_margin_max', 100.0)

        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.image_width = float(self.get_parameter('image_width').value)
        self.image_height = float(self.get_parameter('image_height').value)
        self.confidence_margin_max = float(
            self.get_parameter('confidence_margin_max').value
        )

        # Subscriber to AprilTag detector output
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        # Simple eMDB-friendly publishers
        self.pub_visible = self.create_publisher(
            Bool,
            f'/emdb/perception/tag_{self.target_tag_id}_visible',
            10
        )

        self.pub_id = self.create_publisher(
            Int32,
            '/emdb/perception/apriltag_id',
            10
        )

        self.pub_x = self.create_publisher(
            Float32,
            '/emdb/perception/apriltag_x_normalized',
            10
        )

        self.pub_y = self.create_publisher(
            Float32,
            '/emdb/perception/apriltag_y_normalized',
            10
        )

        self.pub_confidence = self.create_publisher(
            Float32,
            '/emdb/perception/apriltag_confidence',
            10
        )

        self.get_logger().info('AprilTag to eMDB bridge started.')

    def tag_callback(self, msg):
        target_found = False

        for detection in msg.detections:
            tag_id = int(detection.id)

            if tag_id == self.target_tag_id:
                target_found = True

                # Normalize center pixel coordinates from 0.0 to 1.0
                x_norm = float(detection.centre.x) / self.image_width
                y_norm = float(detection.centre.y) / self.image_height

                # Normalize confidence approximately from 0.0 to 1.0
                confidence = float(detection.decision_margin) / self.confidence_margin_max
                confidence = max(0.0, min(confidence, 1.0))

                self.pub_id.publish(Int32(data=tag_id))
                self.pub_x.publish(Float32(data=x_norm))
                self.pub_y.publish(Float32(data=y_norm))
                self.pub_confidence.publish(Float32(data=confidence))

                self.get_logger().info(
                    f'Tag {tag_id} detected: x={x_norm:.3f}, y={y_norm:.3f}, confidence={confidence:.3f}'
                )

                break

        self.pub_visible.publish(Bool(data=target_found))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagToEMDBBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()