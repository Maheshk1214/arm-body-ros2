# Arm + Body: ROS 2 Jazzy, left arm hardware + ArUco tag reach

Fork of [Aqid2108/arm-body-ros2](https://github.com/Aqid2108/arm-body-ros2): a dual-arm robot (two 6-DOF arms + grippers on an aluminium-extrusion body) running in RViz, Gazebo Harmonic and MoveIt 2 on **Ubuntu 24.04 / ROS 2 Jazzy**.

This fork adds:
- **Real arms (A and B)**: MoveIt drives the physical Feetech servos of both arms via `arm_body_hardware`
- **ArUco tag reach**: arm A's gripper camera finds an ArUco tag and moves the jaw tip to it, in simulation or with a real camera

![Arm A kinematic chain](docs/arm_chain.png)

## Demos
Recorded in Gazebo Harmonic (real speed; the dual-arm routine at 1.5x). Play them yourself
with `arm_motion_player/run.sh`, menu **M**.

| Wave | Dual-arm routine | 67 dance |
|:---:|:---:|:---:|
| ![Wave](docs/media/wave.gif) | ![Dual-arm routine](docs/media/dual_arm.gif) | ![67 dance](docs/media/dance67.gif) |

## Quick start
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b feat/arm-motion-demos https://github.com/Maheshk1214/arm-body-ros2.git
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

ros2 launch arm_body_description display.launch.py   # RViz + joint sliders
ros2 launch arm_body_description gazebo.launch.py    # Gazebo + ros2_control
```

Full setup, commands and joint reference: **[docs/RUN_GUIDE.md](docs/RUN_GUIDE.md)**.

## Real arms (A and B)
Each arm has its own Feetech bus (USB board, 1 Mbaud) with servos 1–6 on joints 1–6. `feetech_bridge` drives both, serves MoveIt's `FollowJointTrajectory` actions (`arm_a_controller`, `arm_b_controller`) and publishes `/joint_states`. The grippers have no servo yet and are simulated, as is every joint of an arm left out with `arms:=`, so MoveIt always sees a complete robot. Ports, servo ids and signs are in [`arm_body_hardware/config/arms.yaml`](arm_body_hardware/config/arms.yaml).

```bash
ros2 run arm_body_hardware calibrate --arm arm_a                 # once per arm: record the zero pose (arm hanging straight down)
ros2 run arm_body_hardware calibrate --arm arm_b
ros2 launch arm_body_hardware real_arm.launch.py monitor:=true   # torque off: move by hand, check RViz follows correctly
ros2 launch arm_body_hardware real_arm.launch.py                 # MoveIt drives both real arms
ros2 launch arm_body_hardware real_arm.launch.py arms:=arm_a     # only arm A real, arm B simulated
```
Motion demos run on the real arms too; real servos stop a little short and lag the plan, so pass looser tolerances:
```bash
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[dance67_elbow]" -p speed:=0.675 -p dance_tempo:=67.5 \
    -p dance_amplitude:=0.15 -p repetitions:=2 -p goal_tolerance:=0.05 -p limit_guard:=0.0
```
`calibrate --joints arm_a_joint6` recalibrates only the joints given. Wrist roll (joint 6) has a full-turn URDF range, so the bridge limits it to the part of that turn the servo reaches from its zero.
Don't run it alongside the mock demo or Gazebo, because they use the same topic and controller names.

## ArUco tag reach
`aruco_reach` detects an ArUco tag (DICT_4X4_50, id 0, 50 mm) with the camera on arm A's gripper, estimates its pose and moves the jaw tip in front of it with MoveIt (collision-checked).

```bash
ros2 launch arm_motion_demos aruco_reach_sim.launch.py           # simulated camera + tag in Gazebo
ros2 launch arm_motion_demos aruco_reach_sim.launch.py camera:=stream stream_url:=http://<esp32-ip>:81/stream   # real camera
ros2 run arm_motion_demos aruco_reach                            # second terminal
ros2 run arm_motion_demos aruco_reach --ros-args -p dry_run:=true   # detect + plan only
ros2 launch arm_motion_demos aruco_tf_view.launch.py             # just watch the tag TF in RViz
```
Parameters and details: **[arm_motion_demos/README.md](arm_motion_demos/README.md#aruco-reach-arm-as-gripper-camera)**.

## Contents
| Path | What |
|---|---|
| `arm_body_description/` | URDF/xacro, meshes, controllers, launch files; gripper camera mount (`arm_a_camera.xacro`) |
| `arm_body_hardware/` | **New:** Feetech servo bridge, calibration and launch for the real arms A and B |
| `arm_body_moveit_config/` | MoveIt 2 configuration: mock demo and MoveIt on Gazebo ([README](arm_body_moveit_config/README.md)) |
| `arm_motion_demos/` | Gesture demos (wave, dual-arm, 67 dance) as YAML motion files, sequences, tests, video recording, plus **new** ArUco reach, tag TF viewer and camera streaming ([README](arm_motion_demos/README.md)) |
| `arm_motion_player/` | One-command menu (`./run.sh`): starts Gazebo + MoveIt, single-joint demos, live keyboard control, the motion library ([README](arm_motion_player/README.md)) |
| `docs/RUN_GUIDE.md` | Install, build, RViz, Gazebo, joint/controller reference, troubleshooting |

## Key facts
- 30 links, 14 controllable joints (6 per arm + 1 gripper per arm), 6.82 kg total
- Shoulder servos: Feetech STS3250 (12 V); all others STS3215 (12 V)
- Arm B mirrors arm A: send the same joint values for mirror-image motion
- Self-collision is optional: `gazebo.launch.py self_collide:=true`
- Real hardware status: arm A joints 1–5 connected; wrist roll, gripper and arm B still simulated

## Contributors
| Who | Contribution |
|---|---|
| [Aqid Rafiuddin](https://github.com/Aqid2108) | Original robot description package (URDF, meshes, RViz/Gazebo launch) and run guide |
| [Eileen Teoh Yan Zhen](https://github.com/eileentyz) | MoveIt 2 configuration, MoveIt on Gazebo, gesture demos (wave, dual-arm, 67) and menu player with DOF demos, jog and live control |
| [Mahesh](https://github.com/Maheshk1214) | Real left arm (arm A) Feetech hardware bridge and calibration, gripper camera and ArUco tag reach |
