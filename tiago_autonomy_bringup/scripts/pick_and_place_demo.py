#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TiagoPickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('tiago_pick_and_place_demo')
        
        # Ensure node strictly synchronizes with /clock
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_right_controller/joint_trajectory', 10
        )
        self.torso_pub = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10
        )
        self.head_pub = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/right_robotiq_gripper_controller/joint_trajectory', 10
        )

        self.get_logger().info('Initialized Pick & Place with Sim-Time Synchronization.')

    def sim_sleep(self, seconds: float):
        """Sleeps according to Gazebo /clock time, not system wall time."""
        start = self.get_clock().now()
        duration = Duration(seconds=seconds)
        while rclpy.ok() and (self.get_clock().now() - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_trajectory(self, publisher, joint_names, positions, duration_sec=3.0):
        msg = JointTrajectory()
        # sec=0, nanosec=0 instructs the controller to execute immediately
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        msg.points.append(point)

        for _ in range(5):
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_torso(self, height):
        self.get_logger().info(f'Setting torso height: {height}m')
        self.publish_trajectory(self.torso_pub, ['torso_lift_joint'], [height], duration_sec=3.0)
        self.sim_sleep(3.5)

    def set_head(self, pitch=-0.45):
        self.get_logger().info(f'Tilting head pitch to {pitch} rad')
        self.publish_trajectory(self.head_pub, ['head_1_joint', 'head_2_joint'], [0.0, pitch], duration_sec=2.0)
        self.sim_sleep(2.5)

    def set_gripper(self, open_ratio, duration=1.5):
        gripper_joints = ['gripper_right_finger_joint']
        target_pos = (1.0 - open_ratio) * 0.78
        state_str = "OPEN" if open_ratio > 0.5 else "CLOSE"
        self.get_logger().info(f'Gripper: {state_str} ({target_pos:.3f} rad)')
        self.publish_trajectory(self.gripper_pub, gripper_joints, [target_pos], duration_sec=duration)
        self.sim_sleep(duration + 0.5)

    def set_arm(self, positions, duration=3.5, label=""):
        joints = [
            'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
            'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
        ]
        self.get_logger().info(f'Arm moving to: {label}')
        self.publish_trajectory(self.arm_pub, joints, positions, duration_sec=duration)
        self.sim_sleep(duration + 0.5)

    def run(self):
        # Wait for clock and discovery to connect
        self.get_logger().info('Waiting for /clock and controller discovery...')
        self.sim_sleep(2.0)

        # 1. Lower head to frame the shelf
        self.set_head(-0.45)

        # 2. Torso lift to reach shelf height
        self.set_torso(0.28)

        # 3. Open gripper
        self.set_gripper(open_ratio=1.0)

        # 4. Pre-grasp standoff
        pre_grasp = [0.60, 0.20, -0.10, 1.20, -1.57, 0.50, 0.0]
        self.set_arm(pre_grasp, duration=4.0, label="Pre-grasp standoff")

        # 5. Advance fingers around bottle body
        grasp_contact = [0.85, 0.30, -0.10, 0.95, -1.57, 0.40, 0.0]
        self.set_arm(grasp_contact, duration=3.0, label="Advancing around bottle")

        # 6. Close gripper on bottle
        self.set_gripper(open_ratio=0.0)

        # 7. Lift off shelf
        lift_pose = [0.85, 0.05, -0.10, 0.95, -1.57, 0.40, 0.0]
        self.set_arm(lift_pose, duration=3.0, label="Lifting bottle")

        # 8. Place on side target
        place_pose = [0.35, -0.30, -0.20, 1.40, -1.57, 0.80, 0.0]
        self.set_arm(place_pose, duration=4.0, label="Placement position")

        # 9. Release gripper
        self.set_gripper(open_ratio=1.0)

        # 10. Home position
        home_pose = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        self.set_arm(home_pose, duration=4.0, label="Home")

        self.get_logger().info('Demo execution completed successfully!')

def main(args=None):
    rclpy.init(args=args)
    demo = TiagoPickAndPlaceDemo()
    try:
        demo.run()
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()