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

## Run with Gazebo
```bash
ros2 launch arm_body_moveit_config moveit_gazebo.launch.py     # Gazebo + MoveIt + RViz in one go
```
or, with Gazebo already running from `ros2 launch arm_body_description gazebo.launch.py`:
```bash
ros2 launch arm_body_moveit_config move_group_gazebo.launch.py
```
Plan in RViz as above; **Plan & Execute** moves the simulated robot through the existing
`arm_a_controller`, `arm_b_controller`, `gripper_a_controller` and `gripper_b_controller`
(`<controller>/follow_joint_trajectory`). A `both_arms` plan is split across the two arm controllers.
Launch arguments: `self_collide` (must match Gazebo), `rviz`, `ground_plane`, and for
`moveit_gazebo.launch.py` also `world`.

How it fits together:
- Gazebo's launch owns `robot_state_publisher`, the `controller_manager` (gz_ros2_control) and
  `/joint_states`. The MoveIt launches add only `move_group`, RViz and the ground plane, all on sim time.
- `check_sim_controllers.py` gates start-up. It refuses if a `move_group` or the mock
  `ros2_control_node` is already running on the same `ROS_DOMAIN_ID`, or if the `controller_manager`'s
  hardware isn't Gazebo's. Otherwise it waits until all five controllers are active, because MoveIt drops
  controllers whose action servers aren't up when `move_group` starts. Don't run the mock demo and Gazebo
  on the same domain.
- `add_ground_plane.py` adds Gazebo's floor (top at z = 0) to the planning scene. At `home` the arms
  hang only ~2 cm above it.
- Holding a joint exactly on its limit jams it in Gazebo (gravity pushes it just past the limit and
  DART then ignores the position command). joint2 (max 0), joint5, joint6 and the grippers (min 0)
  have a limit at 0, so `home` and `closed` sit 0.01 rad inside those limits, and
  `config/gazebo_execution.yaml` (Gazebo only) narrows MoveIt's copy of them by 5e-3 rad. A goal on
  such a limit (e.g. 0 in RViz) is clamped to the narrowed bound, so MoveIt never plans onto it. A
  start state within 5e-3 rad of the limit is rejected as out of bounds. Commands that bypass MoveIt
  (e.g. a trajectory topic to an all-zero pose) can still jam the arm.

Check the whole chain (it moves the simulated robot):
```bash
ros2 run arm_body_moveit_config check_gazebo_execution.py
```
The Gazebo controllers have `goal_time: 0.0` and no goal tolerances, so they report success even when
a joint doesn't reach its goal. This check reads `/joint_states` instead. It also holds `home` for 40 s of sim time and moves away
again, and checks that all-zero goals are planned no closer than 5e-3 rad to the limits.

## Planning groups
| Group | Contents | Named states |
|---|---|---|
| `arm_a` | chain `arm_a_servo_base` → `arm_a_gripper_1` (joints 1–6) | `home` (joint2 −0.01, joint5/6 0.01, rest 0), `elbow_90` (upper arm down, elbow ~90°, gripper level) |
| `arm_b` | chain `arm_b_servo_base` → `arm_b_gripper_1` (joints 1–6) | `home` |
| `both_arms` | `arm_a` + `arm_b` (12 joints) | `home` |
| `gripper_a` / `gripper_b` | `arm_?_gripper_joint` (0 = closed limit) | `open` (1.2 rad), `closed` (0.01) |

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
| `launch/demo.launch.py` | Full RViz demo (mock hardware) |
| `launch/moveit_gazebo.launch.py` | Gazebo + MoveIt + RViz |
| `launch/move_group_gazebo.launch.py` | MoveIt + RViz attached to a running Gazebo |
| `config/gazebo_execution.yaml` | Gazebo-only execution settings and limit margins |
| `scripts/check_sim_controllers.py` | Start-up guard: single Gazebo controller_manager, controllers active |
| `scripts/add_ground_plane.py` | Adds Gazebo's floor to the planning scene |
| `scripts/check_gazebo_execution.py` | End-to-end execution check against Gazebo |
| `launch/setup_assistant.launch.py` | Reopen this package in MoveIt Setup Assistant |
