# Arm + Body: ROS 2 Jazzy & Gazebo Run Guide

Oct 17, 2026 · Aqid Rafiuddin

## Overview

The `arm_body_description` package runs the dual-arm robot in RViz and in Gazebo Harmonic on ROS 2 Jazzy, with all 14 servo joints controllable through ros2\_control. It was generated directly from the Fusion design **Arm + Body v15**, so the meshes, masses, inertias, joint positions and joint limits all come from the CAD.

| Item | Value |
| --- | --- |
| Links | 30 (incl. `world`) |
| Moving joints | 14 revolute (6 per arm + 1 gripper per arm) |
| Fixed joints | 15 (servos to brackets, servo bases to body, body to world) |
| Total mass | 6.82 kg (body 5.54 kg, each arm 637 g) |
| Shoulder servos (joint1, joint2) | Feetech STS3250, 12 V |
| All other servos | Feetech STS3215, 12 V |
| Printed parts | PETG, 15% infill (31–35% of solid density incl. walls) |
| Body | Aluminium 6061, fixed to the world |

![Arm A kinematic chain: 9 links, 8 joints](arm_chain.png)

Each servo is fixed to the bracket it rides on, so a servo and its bracket move as one link. Joint names in the picture are the ones you command in section 4.

Clone this repository into a ROS 2 workspace `src` folder (section 2).

## 1. Install ROS 2 Jazzy and Gazebo Harmonic

You need Ubuntu 24.04 (Noble), ROS 2 Jazzy desktop, Gazebo Harmonic and the ros2\_control packages. Skip step 1.1 if Jazzy is already installed (`ls /opt/ros` shows `jazzy`).

**1.1 ROS 2 Jazzy** (from the [official install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)):

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

**1.2 Gazebo Harmonic, ros2\_control and viewer tools.** Jazzy pairs with Gazebo Harmonic, and `ros-jazzy-ros-gz` installs it:

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui
```

**1.3 Source ROS in every new terminal** (add it to `~/.bashrc` so it happens automatically):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check: `gz sim --version` should print a Gazebo Sim 8.x version (Harmonic).

## 2. Copy the package to Ubuntu and build it

Clone the repository into a new workspace (or copy `arm_body_ws.zip` from the Windows PC and unzip it to `~/arm_body_ws` instead), then build:

```bash
mkdir -p ~/arm_body_ws/src && cd ~/arm_body_ws/src
git clone https://github.com/Aqid2108/arm-body-ros2.git
```

```bash
cd ~/arm_body_ws
sudo rosdep init                # first time on this machine only; ignore 'already exists'
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Run `source ~/arm_body_ws/install/setup.bash` in every new terminal before using the package, or add it to `~/.bashrc` the same way as step 1.3.

Check the URDF is valid:

```bash
ros2 run xacro xacro $(ros2 pkg prefix arm_body_description)/share/arm_body_description/urdf/arm_body.urdf.xacro > /tmp/arm_body.urdf
check_urdf /tmp/arm_body.urdf   # sudo apt install liburdfdom-tools if missing
```

`check_urdf` should report root link `world` and list all 30 links in one tree.

## 3. View the robot in RViz

RViz shows the robot with a slider for every joint and no physics, so it is the quickest check that the model looks right.

```bash
ros2 launch arm_body_description display.launch.py
```

- The **Joint State Publisher** window has one slider per moving joint. Drag `arm_a_joint2` to swing arm A's shoulder.
- Fixed Frame is `world`. The body sits on the grid with its base at z = 0.
- To see the collision meshes, expand **RobotModel** in the Displays panel and tick **Collision Enabled**.

## 4. Run it in Gazebo and move the arms

One launch file starts Gazebo, spawns the robot fixed to the ground, and starts five controllers once the robot exists.

```bash
ros2 launch arm_body_description gazebo.launch.py
# or with RViz alongside:
ros2 launch arm_body_description gazebo.launch.py rviz:=true
# with self-collision (arms collide with the body and each other):
ros2 launch arm_body_description gazebo.launch.py self_collide:=true
```

**4.1 Check the controllers are active** (second terminal, after sourcing both setup files):

```bash
ros2 control list_controllers
```

You should see `joint_state_broadcaster`, `arm_a_controller`, `arm_b_controller`, `gripper_a_controller` and `gripper_b_controller`, all `active`.

**4.2 Raise arm A forward and outward** (reaches the pose in 3 s):

```bash
ros2 topic pub --once /arm_a_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [arm_a_joint1, arm_a_joint2, arm_a_joint3, arm_a_joint4, arm_a_joint5, arm_a_joint6],
  points: [{positions: [0.3, -1.2, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"
```

**4.3 Mirror-image pose on arm B.** Arm B is set up to mirror arm A, so **send the same numbers** with `arm_b_` joint names:

```bash
ros2 topic pub --once /arm_b_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [arm_b_joint1, arm_b_joint2, arm_b_joint3, arm_b_joint4, arm_b_joint5, arm_b_joint6],
  points: [{positions: [0.3, -1.2, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"
```

**4.4 Open both grippers** (same value on both):

```bash
ros2 topic pub --once /gripper_a_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [arm_a_gripper_joint], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
ros2 topic pub --once /gripper_b_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [arm_b_gripper_joint], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
```

**4.5 Back to the hanging pose:** send `positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` to each arm controller.

**4.6 Watch the joint angles:** `ros2 topic echo /joint_states`.

Angles are in radians (1.57 = 90°). A goal outside a joint's limits (section 5) is rejected by the controller.

## 5. Joint and controller reference

Every revolute joint carries the servo's stall torque as `effort`, its no-load speed as `velocity`, and `<dynamics damping="0.1" friction="0.05"/>`. Zero on every joint is the pose modelled in Fusion: both arms hanging straight down.

| Joint | Moves | Servo (12 V) | Range (deg) | Effort (N·m) | Velocity (rad/s) | Controller |
| --- | --- | --- | --- | --- | --- | --- |
| `arm_a_joint1` | shoulder yaw | STS3250 | −90 to 90 | 4.90 | 7.85 | `arm_a_controller` |
| `arm_a_joint2` | shoulder pitch | STS3250 | −180 to 0 | 4.90 | 7.85 | `arm_a_controller` |
| `arm_a_joint3` | upper arm | STS3215 | −90 to 90 | 2.94 | 4.72 | `arm_a_controller` |
| `arm_a_joint4` | elbow | STS3215 | −90 to 90 | 2.94 | 4.72 | `arm_a_controller` |
| `arm_a_joint5` | wrist pitch | STS3215 | 0 to 180 | 2.94 | 4.72 | `arm_a_controller` |
| `arm_a_joint6` | wrist roll | STS3215 | 0 to 360 | 2.94 | 4.72 | `arm_a_controller` |
| `arm_a_gripper_joint` | gripper jaw | STS3215 | 0 to 90 | 2.94 | 4.72 | `gripper_a_controller` |
| `arm_b_joint1` | shoulder yaw | STS3250 | −90 to 90 | 4.90 | 7.85 | `arm_b_controller` |
| `arm_b_joint2` | shoulder pitch | STS3250 | −180 to 0 | 4.90 | 7.85 | `arm_b_controller` |
| `arm_b_joint3` | upper arm | STS3215 | −90 to 90 | 2.94 | 4.72 | `arm_b_controller` |
| `arm_b_joint4` | elbow | STS3215 | −90 to 90 | 2.94 | 4.72 | `arm_b_controller` |
| `arm_b_joint5` | wrist pitch | STS3215 | 0 to 180 | 2.94 | 4.72 | `arm_b_controller` |
| `arm_b_joint6` | wrist roll | STS3215 | 0 to 360 | 2.94 | 4.72 | `arm_b_controller` |
| `arm_b_gripper_joint` | gripper jaw | STS3215 | 0 to 90 | 2.94 | 4.72 | `gripper_b_controller` |

Servo specs: STS3250 stall 50 kg·cm, rated 16 kg·cm, 0.133 s/60°, 74.5 g ([Evelta](https://evelta.com/sts3250-12v-50kg-360deg-magnetic-encoder-double-shaft-ttl-servo/)); STS3215 stall 30 kg·cm, rated 10 kg·cm, 0.222 s/60°, 55 g ([Eckstein](https://eckstein-shop.de/Motors?mf=1602)).

**Arm B mirrors arm A:** the same value on the same joint number gives mirror-image motion, and both arms share the same ranges. In Fusion, arm B's joint2, joint3, joint4 and gripper axes point the opposite way, so the package flips them. The ranges for joint6 (0–360° on both arms) are also set in the package; arm A's joint6 is still locked at 0° in Fusion.

Arm B's CAD is not an exact mirror copy of arm A. Its shoulder-yaw pivot and its gripper/joint6 assembly sit slightly differently, so with the same joint1 or joint6 value, arm B's gripper lands a few centimetres from the exact mirror point. Joints 2–5 and the gripper mirror exactly.

| File | What it holds |
| --- | --- |
| `urdf/arm_body.urdf` | Plain URDF: links, joints, meshes, masses, limits (no ros2\_control) |
| `urdf/arm_body.urdf.xacro` | Main file used by both launch files: the URDF plus ros2\_control and the Gazebo plugin |
| `config/controllers.yaml` | Controller manager at 100 Hz, joint\_state\_broadcaster, 4 JointTrajectoryControllers (position) |
| `launch/display.launch.py` | RViz + joint sliders |
| `launch/gazebo.launch.py` | Gazebo Harmonic + spawn + controllers (`rviz:=true`, `world:=<file>`) |

## 6. Troubleshooting and known limitations

| Symptom | Fix |
| --- | --- |
| `Package 'arm_body_description' not found` | Run `source ~/arm_body_ws/install/setup.bash` in that terminal. |
| Gazebo shows no robot, or grey boxes instead of meshes | Rebuild with `colcon build --symlink-install`, re-source, relaunch. The launch file adds the package to `GZ_SIM_RESOURCE_PATH` so `package://` mesh paths resolve. |
| `list_controllers` shows nothing, or the spawner waits forever | Check `sudo apt install ros-jazzy-gz-ros2-control` succeeded, then look for `gz_ros2_control` errors in the launch terminal. |
| A trajectory is ignored | Joint names must match section 5 exactly, and every position must be inside the joint's range. |
| Gazebo window is black or crashes in a VM / WSL | Run `export LIBGL_ALWAYS_SOFTWARE=1` before launching. |
| `ros2 control` command not found | `sudo apt install ros-jazzy-ros2controlcli` |

**Known limitations**

- **Built and checked here, not yet run on Jazzy.** The URDF was expanded with xacro, checked as one connected tree with valid inertias, and rendered in a test pose. The first real Gazebo run is the final check.
- **Simulation is idealised.** ros2\_control in Gazebo drives joints straight to the commanded angle, so the servo torque limits are recorded but not enforced like a real stall. The shoulder needs about 14.6 kg·cm to hold arm A straight out, within the STS3250's 50 kg·cm stall.
- **Self-collision is off**, so an arm can pass through the body in simulation. Launch with self\_collide:=true to turn it on. Collision with the ground and other objects in the world is always on.
- **Printed-part masses are estimates** for 15% infill with about 1.2 mm of solid walls. Weigh a printed part to refine them.
- **Collision meshes are the full visual meshes.** If Gazebo runs slowly, replace them with boxes or cylinders in `arm_body.urdf.xacro`.
- **CAD changes are not picked up automatically.** If the Fusion model changes, the package must be regenerated.

## Sources

- [ROS 2 Jazzy: Ubuntu (deb packages) install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [gz\_ros2\_control for Jazzy (control.ros.org)](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Feetech STS3250 12 V specs (Evelta)](https://evelta.com/sts3250-12v-50kg-360deg-magnetic-encoder-double-shaft-ttl-servo/)
- [Feetech STS3215 12 V specs (Eckstein)](https://eckstein-shop.de/Motors?mf=1602)
