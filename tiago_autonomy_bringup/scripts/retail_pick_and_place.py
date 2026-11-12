#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from control_msgs.action import ParallelGripperCommand
import tf2_ros


class RetailPickAndPlace(Node):
    def __init__(self):
        super().__init__('retail_pick_and_place')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, ParallelGripperCommand, '/right_robotiq_gripper_controller/gripper_cmd'
        )
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def wait_for_connections(self):
        self.get_logger().info('Connecting to MoveIt 2 and robot interfaces...')
        while not self.move_group_client.wait_for_server(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        while not self.gripper_client.wait_for_server(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        while not self.cartesian_client.wait_for_service(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        while not self.exec_client.wait_for_server(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        while not self.apply_scene_client.wait_for_service(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('All MoveIt 2 interfaces connected successfully.')

    def get_current_ee_pose(self):
        """Looks up the live transform of arm_right_tool_link relative to base_footprint."""
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', 'arm_right_tool_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception as e:
            self.get_logger().error(f'Failed to lookup current EE pose: {e}')
            return None

    def build_shelf_scene(self, bottle_x=0.72, bottle_y=-0.10, bottle_z=1.05):
        """Registers Shelf 2 Level 3 with deep floor and elevated ceiling margins."""
        req = ApplyPlanningScene.Request()
        scene = PlanningScene()
        scene.is_diff = True

        # Level 3 Shelf Board (Floor lowered)
        shelf_lower = CollisionObject()
        shelf_lower.header.frame_id = 'base_footprint'
        shelf_lower.id = 'shelf_level_3_floor'
        box_lower = SolidPrimitive()
        box_lower.type = SolidPrimitive.BOX
        box_lower.dimensions = [0.60, 1.0, 0.03]
        pose_lower = Pose()
        pose_lower.position.x = bottle_x + 0.10
        pose_lower.position.y = bottle_y
        pose_lower.position.z = bottle_z - 0.20
        pose_lower.orientation.w = 1.0
        shelf_lower.primitives.append(box_lower)
        shelf_lower.primitive_poses.append(pose_lower)
        shelf_lower.operation = CollisionObject.ADD

        # Level 4 Shelf Board (Ceiling / Overhang)
        shelf_upper = CollisionObject()
        shelf_upper.header.frame_id = 'base_footprint'
        shelf_upper.id = 'shelf_level_4_ceiling'
        box_upper = SolidPrimitive()
        box_upper.type = SolidPrimitive.BOX
        box_upper.dimensions = [0.60, 1.0, 0.03]
        pose_upper = Pose()
        pose_upper.position.x = bottle_x + 0.10
        pose_upper.position.y = bottle_y
        pose_upper.position.z = bottle_z + 0.45
        pose_upper.orientation.w = 1.0
        shelf_upper.primitives.append(box_upper)
        shelf_upper.primitive_poses.append(pose_upper)
        shelf_upper.operation = CollisionObject.ADD

        # Target finish_protector Object
        target_obj = CollisionObject()
        target_obj.header.frame_id = 'base_footprint'
        target_obj.id = 'finish_protector'
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.18, 0.035]
        pose_cyl = Pose()
        pose_cyl.position.x = bottle_x
        pose_cyl.position.y = bottle_y
        pose_cyl.position.z = bottle_z
        pose_cyl.orientation.w = 1.0
        target_obj.primitives.append(cyl)
        target_obj.primitive_poses.append(pose_cyl)
        target_obj.operation = CollisionObject.ADD

        scene.world.collision_objects = [shelf_lower, shelf_upper, target_obj]
        req.scene = scene

        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Planning scene obstacles and finish_protector registered.')

    def attach_bottle(self, obj_id='finish_protector', attach=True):
        req = ApplyPlanningScene.Request()
        scene = PlanningScene()
        scene.is_diff = True

        attached = AttachedCollisionObject()
        attached.link_name = 'arm_right_tool_link'
        attached.object.id = obj_id
        
        attached.touch_links = [
            'arm_right_tool_link',
            'gripper_right_base_link',
            'gripper_right_coupler_link',
            'gripper_right_left_finger_pad',
            'gripper_right_right_finger_pad',
            'gripper_right_left_inner_finger',
            'gripper_right_right_inner_finger',
        ]
        
        if attach:
            attached.object.operation = CollisionObject.ADD
            cyl = SolidPrimitive()
            cyl.type = SolidPrimitive.CYLINDER
            cyl.dimensions = [0.18, 0.035]
            pose_cyl = Pose()
            pose_cyl.orientation.w = 1.0
            attached.object.primitives.append(cyl)
            attached.object.primitive_poses.append(pose_cyl)
            attached.object.header.frame_id = 'arm_right_tool_link'
            scene.robot_state.attached_collision_objects.append(attached)

            world_obj = CollisionObject()
            world_obj.header.frame_id = 'base_footprint'
            world_obj.id = obj_id
            world_obj.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(world_obj)
        else:
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)

        scene.robot_state.is_diff = True
        req.scene = scene

        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Object {obj_id} {"attached" if attach else "detached"}.')

    def set_gripper(self, position: float):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(position)]

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Gripper command rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info(f'Gripper position: {position}')
        return True

    def move_to_pose(self, target_pose: Pose, group_name='arm_right_torso'):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4

        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'base_footprint'
        pos_con.link_name = 'arm_right_tool_link'
        bound = SolidPrimitive()
        bound.type = SolidPrimitive.SPHERE
        bound.dimensions = [0.03]
        pos_con.constraint_region.primitives.append(bound)
        pos_con.constraint_region.primitive_poses.append(target_pose)
        pos_con.weight = 1.0

        orient_con = OrientationConstraint()
        orient_con.header.frame_id = 'base_footprint'
        orient_con.link_name = 'arm_right_tool_link'
        orient_con.orientation = target_pose.orientation
        orient_con.absolute_x_axis_tolerance = 0.15
        orient_con.absolute_y_axis_tolerance = 0.15
        orient_con.absolute_z_axis_tolerance = 0.15
        orient_con.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(orient_con)
        req.goal_constraints.append(constraints)

        goal.request = req
        goal.planning_options.plan_only = False

        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Motion plan goal rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def execute_cartesian_delta(self, dx=0.0, dy=0.0, dz=0.0):
        current_pose = self.get_current_ee_pose()
        if not current_pose:
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_footprint'
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = 'arm_right_torso'
        req.link_name = 'arm_right_tool_link'
        req.start_state.is_diff = True

        next_pose = Pose()
        next_pose.position.x = current_pose.position.x + dx
        next_pose.position.y = current_pose.position.y + dy
        next_pose.position.z = current_pose.position.z + dz
        next_pose.orientation = current_pose.orientation

        req.waypoints = [next_pose]
        req.max_step = 0.01
        req.jump_threshold = 2.5
        req.avoid_collisions = True

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.fraction < 0.8:
            self.get_logger().warn(f'Cartesian fraction low: {res.fraction:.2f}')
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        send_future = self.exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)

        return True


def main(args=None):
    rclpy.init(args=args)
    demo = RetailPickAndPlace()
    wait_start = time.time()
    while time.time() - wait_start < 2.0:
        rclpy.spin_once(demo, timeout_sec=0.1)

    demo.wait_for_connections()

    BOTTLE_X = 0.72
    BOTTLE_Y = -0.10
    BOTTLE_Z = 1.05

    # 1. Build Scene
    demo.build_shelf_scene(BOTTLE_X, BOTTLE_Y, BOTTLE_Z)

    # 2. Open Gripper
    demo.get_logger().info('Phase 1: Opening Gripper...')
    demo.set_gripper(0.0)

    # Neutral wrist orientation
    fwd_orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)

    # 3. Plan directly to pre-grasp standoff (optimized distance so arm reaches without base motion)
    demo.get_logger().info('Phase 2: Moving directly to pre-grasp standoff...')
    standoff_pose = Pose()
    standoff_pose.position.x = BOTTLE_X - 0.18
    standoff_pose.position.y = BOTTLE_Y
    standoff_pose.position.z = BOTTLE_Z + 0.05
    standoff_pose.orientation = fwd_orientation

    if not demo.move_to_pose(standoff_pose):
        demo.get_logger().error('Failed to reach pre-grasp standoff.')
        demo.destroy_node()
        rclpy.shutdown()
        return

    # Allow TF buffer and controller to fully settle
    time.sleep(1.0)

    # 4. Short Straight Cartesian Insertion Into Shelf (6cm push into bottle)
    demo.get_logger().info('Phase 3: Linear Insertion into Shelf...')
    if not demo.execute_cartesian_delta(dx=0.06):
        demo.get_logger().error('Cartesian insertion failed!')
        demo.destroy_node()
        rclpy.shutdown()
        return

    # 5. Close Gripper & Attach Object
    demo.get_logger().info('Phase 4: Grasping Object...')
    demo.set_gripper(0.75)
    time.sleep(1.0)
    demo.attach_bottle(attach=True)

    # 6. Vertical Lift
    demo.get_logger().info('Phase 5: Vertical Lift...')
    demo.execute_cartesian_delta(dz=0.06)

    # 7. Linear Retract Out of Shelf
    demo.get_logger().info('Phase 6: Linear Retraction...')
    demo.execute_cartesian_delta(dx=-0.12)

    # 8. Transfer to Drop-off Position
    demo.get_logger().info('Phase 7: Transferring to Drop-off...')
    dropoff_pose = Pose()
    dropoff_pose.position.x = 0.45
    dropoff_pose.position.y = 0.35
    dropoff_pose.position.z = 0.80
    dropoff_pose.orientation = fwd_orientation
    demo.move_to_pose(dropoff_pose)

    # 9. Release
    demo.get_logger().info('Phase 8: Releasing Object...')
    demo.attach_bottle(attach=False)
    demo.set_gripper(0.0)

    demo.get_logger().info('Pick-and-place scenario for finish_protector completed successfully!')
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()