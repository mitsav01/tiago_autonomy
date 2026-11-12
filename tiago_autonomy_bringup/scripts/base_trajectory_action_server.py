#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
import tf2_ros
from tf_transformations import euler_from_quaternion


class BaseTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('base_trajectory_action_server')

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mobile_base_controller/cmd_vel',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/base_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.k_v = 1.5
        self.k_w = 2.5
        self.get_logger().info('BaseTrajectoryActionServer ready on /base_controller/follow_joint_trajectory.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Accepted base trajectory goal from MoveIt.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = 'base_footprint'
        self.cmd_vel_pub.publish(stop_cmd)

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        try:
            x_idx = joint_names.index('virtual_joint/x')
            y_idx = joint_names.index('virtual_joint/y')
            yaw_idx = (
                joint_names.index('virtual_joint/theta')
                if 'virtual_joint/theta' in joint_names
                else None
            )
        except ValueError:
            self.get_logger().error(f'Missing virtual_joint joints in trajectory: {joint_names}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        start_time = self.get_clock().now()
        last_pt = trajectory.points[-1]
        total_duration = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9

        rate = self.create_rate(30)  # 30 Hz loop
        self.get_logger().info(f'Executing base trajectory: {len(trajectory.points)} points over {total_duration:.2f}s')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                res = FollowJointTrajectory.Result()
                res.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return res

            now_sec = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if now_sec >= total_duration:
                break

            # Find target point along the timeline
            target_pt = trajectory.points[0]
            for pt in trajectory.points:
                pt_time = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                if pt_time >= now_sec:
                    target_pt = pt
                    break

            target_x = target_pt.positions[x_idx]
            target_y = target_pt.positions[y_idx]

            # Read current base odometry position
            try:
                # Use lookup with 0 duration to avoid stalling the thread
                t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                curr_x = t.transform.translation.x
                curr_y = t.transform.translation.y
                q = t.transform.rotation
                _, _, curr_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            except Exception as e:
                # Still output feedforward command so the controller does not time out
                curr_x, curr_y, curr_yaw = target_x, target_y, 0.0

            target_yaw = (
                target_pt.positions[yaw_idx]
                if yaw_idx is not None
                else math.atan2(target_y - curr_y, max(0.01, target_x - curr_x))
            )

            dx = target_x - curr_x
            dy = target_y - curr_y

            # Error projected into robot frame
            err_x = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            err_y = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            err_yaw = math.atan2(math.sin(target_yaw - curr_yaw), math.cos(target_yaw - curr_yaw))

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_footprint'

            v_lin = self.k_v * err_x
            v_ang = self.k_w * err_yaw + (1.5 * math.atan2(err_y, max(0.1, abs(err_x))))

            cmd.twist.linear.x = float(max(-0.6, min(0.6, v_lin)))
            cmd.twist.angular.z = float(max(-1.0, min(1.0, v_ang)))

            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()
        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Base trajectory completed successfully.')
        return res


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = BaseTrajectoryActionServer()
    # MultiThreadedExecutor allows TF buffer updates to run in parallel with execute_callback
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()