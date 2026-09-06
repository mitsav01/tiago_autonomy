#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


class DockPosePublisher(Node):

    def __init__(self):
        super().__init__("dock_pose_publisher")

        self.target_frame = "odom"
        self.tag_frame = "dock_apriltag"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseStamped, "/detected_dock_pose", 10)

        self.timer = self.create_timer(0.05, self.publish_pose)

    def publish_pose(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.tag_frame, Time()
            )

        except TransformException:
            return

        pose = PoseStamped()

        pose.header = tf.header

        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z

        pose.pose.orientation = tf.transform.rotation

        self.publisher.publish(pose)


def main():

    rclpy.init()

    node = DockPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
