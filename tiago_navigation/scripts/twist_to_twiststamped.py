#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStampedRelay(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped_relay')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('input_topic', '/dock_cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_smoothed')
        self.declare_parameter('frame_id', 'base_footprint')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.callback, 10)

        self.get_logger().info(f'Relaying {in_topic} (Twist) -> {out_topic} (TwistStamped)')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()