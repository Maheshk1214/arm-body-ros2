# arm_body_moveit_config

MoveIt 2 configuration for the dual-arm Arm + Body robot. ROS 2 Jazzy, MoveIt 2.12.

## Run
```bash
sudo apt install ros-jazzy-moveit   # once, if MoveIt is not installed
colcon build --symlink-install && source install/setup.bash
ros2 launch arm_body_moveit_config demo.launch.py        # move_group + RViz + mock hardware
```
In the **MotionPlanning** panel, choose **Planning Group** `arm_a`, `arm_b` or `both_arms` (or `gripper_a` / `gripper_b`),
drag the goal marker or pick a named goal state (`home`, `open`, `closed`), then click **Plan** or **Plan & Execute**.
Execution runs on `mock_components/GenericSystem`, not on Gazebo.

## Planning groups
| Group | Contents | Named states |
|---|---|---|
| `arm_a` | chain `arm_a_servo_base` → `arm_a_gripper_1` (joints 1–6) | `home` |
| `arm_b` | chain `arm_b_servo_base` → `arm_b_gripper_1` (joints 1–6) | `home` |
| `both_arms` | `arm_a` + `arm_b` (12 joints) | `home` |
| `gripper_a` / `gripper_b` | `arm_?_gripper_joint` (0 = closed) | `open` (1.2 rad), `closed` (0) |

End effectors: `arm_a_ee` / `arm_b_ee`, attached to `arm_?_gripper_1`.

## Kinematics
Joints 2–5 of each arm rotate about parallel axes, so each arm has only 5 task-space DOF.
KDL runs with `position_only_ik: true`: the goal marker sets the gripper position, and the orientation follows from it.

## Files
| Path | What |
|---|---|
| `config/arm_body.urdf.xacro` | `arm_body_core.xacro` + mock ros2_control (Gazebo xacro unchanged) |
| `config/arm_body.srdf` | Groups, end effectors, named states, self-collision matrix |
| `config/kinematics.yaml` | KDL, position-only IK |
| `config/ompl_planning.yaml` | OMPL pipeline, RRTConnect default |
| `config/joint_limits.yaml` | Velocity (from URDF) and placeholder acceleration limits |
| `config/moveit_controllers.yaml` | MoveIt → `arm_?_controller`, `gripper_?_controller` |
| `config/ros2_controllers.yaml` | Controllers for the mock-hardware demo |
| `launch/demo.launch.py` | Full RViz demo |
| `launch/setup_assistant.launch.py` | Reopen this package in MoveIt Setup Assistant |
