# arm_body_description

Dual-arm robot (Arm + Body v12), generated from Fusion. ROS 2 Jazzy + Gazebo Harmonic, Ubuntu 24.04.

## Build
```bash
cd ~/arm_body_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch arm_body_description display.launch.py          # RViz + joint sliders
ros2 launch arm_body_description gazebo.launch.py           # Gazebo + ros2_control
ros2 launch arm_body_description gazebo.launch.py rviz:=true
ros2 launch arm_body_description gazebo.launch.py self_collide:=true   # arms collide with body/each other
```

## Move an arm (Gazebo running)
```bash
ros2 topic pub --once /arm_a_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [arm_a_joint1, arm_a_joint2, arm_a_joint3, arm_a_joint4, arm_a_joint5, arm_a_joint6],
  points: [{positions: [0.3, -1.2, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"
```
Arm B mirrors arm A: send the same numbers to `/arm_b_controller` with `arm_b_` joint names.

## Files
| Path | What |
|---|---|
| `urdf/arm_body.urdf` | Plain URDF (no ros2_control) |
| `urdf/arm_body_core.xacro` | Same links/joints, included by the main xacro |
| `urdf/arm_body.urdf.xacro` | Main file: core + ros2_control + Gazebo plugin |
| `config/controllers.yaml` | joint_state_broadcaster + 4 JointTrajectoryControllers |
| `launch/display.launch.py` | RViz viewer |
| `launch/gazebo.launch.py` | Gazebo sim |
| `meshes/*.stl` | One mesh per Fusion component (mm, scaled 0.001) |

Joints, limits and servo specs: see the run guide.
