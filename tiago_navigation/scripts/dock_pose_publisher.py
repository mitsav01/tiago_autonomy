#!/usr/bin/env python3

import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__("dock_pose_publisher")

        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("tag_frame", "tag36h11:0")
        self.declare_parameter("publish_rate", 20.0)

        # Tag measured by camera is at ~5.72m.
        # Desired contact resting point for base_footprint is at Y = 4.52m:
        # Offset along Y = 4.52 - 5.72 = -1.20m
        self.declare_parameter("tag_to_dock_y_offset", -1.38)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.tag_frame = self.get_parameter("tag_frame").value
        self.offset_y = self.get_parameter("tag_to_dock_y_offset").value
        rate = self.get_parameter("publish_rate").value

        self.pose_pub = self.create_publisher(PoseStamped, "/detected_dock_pose", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Fallback values
        self.last_tag_x = 8.14
        self.last_tag_y = 4.52

        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.timer_callback)
        self.get_logger().info("DockPosePublisher running...")

    def timer_callback(self):
        now = self.get_clock().now()

        # Update tag position from TF if available
        if self.tf_buffer.can_transform(
            self.fixed_frame, self.tag_frame, Time(), timeout=Duration(seconds=0.02)
        ):
            try:
                tf_tag = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    self.tag_frame,
                    Time(),
                    timeout=Duration(seconds=0.03),
                )
                self.last_tag_x = tf_tag.transform.translation.x
                self.last_tag_y = tf_tag.transform.translation.y
            except TransformException:
                pass

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.fixed_frame

        # Center on tag X (8.07 - 8.14) and offset Y to 4.52m
        pose.pose.position.x = self.last_tag_x
        pose.pose.position.y = self.last_tag_y + self.offset_y
        pose.pose.position.z = 0.0

        # Normal facing outward along +Y into the dock
        # Yaw = +1.5708 rad (pi / 2) -> qz = sin(pi/4) = 0.7071068, qw = cos(pi/4) = 0.7071068
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.7071068
        pose.pose.orientation.w = 0.7071068

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
