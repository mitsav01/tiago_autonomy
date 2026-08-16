# TIAGO-X: Autonomous Mobile Manipulation Platform

TIAGO-X is an end-to-end ROS 2 robotics platform for autonomous mobile manipulation.

The project integrates:

- Autonomous navigation
- Object perception
- 3D pose estimation
- Collision-free motion planning
- Arm and gripper control
- Task-level autonomy
- Action verification

The system is developed in simulation using a TIAGo mobile manipulator and follows a modular architecture designed with sim-to-real transfer in mind.

## Demonstration

The final system enables tasks such as:

> "Find the red cup, navigate to the table, pick it up, and place it at the target location."

The robot autonomously:

1. Perceives the environment.
2. Detects the target object.
3. Estimates its position.
4. Navigates to the object.
5. Plans a collision-free manipulation trajectory.
6. Grasps the object.
7. Transports it to the destination.
8. Places the object.
9. Verifies task completion.

## Architecture

The system consists of five primary layers:

1. Task and autonomy layer
2. Semantic world model
3. Perception and navigation
4. Motion planning
5. Robot control and simulation
