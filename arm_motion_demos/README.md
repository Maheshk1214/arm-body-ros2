# arm_motion_demos

Gesture demonstrations for the dual-arm robot, in simulation (MoveIt mock demo or Gazebo):

| Gesture | What it does |
|---|---|
| `wave` | One arm swings out to the side, forearm up, gripper open; the hand sways ~20 cm side to side |
| `dual_arm` | Both arms forward → raised together → alternating up/down → out to the sides → symmetric flaps |
| `dance67` | "6-7": forearms forward, palms (grippers) open, hands bobbing up and down in turn; the wrists counter-rotate so the palms stay level |
| `dance67_elbow` | The same 6-7, bent at the elbow (joint4) instead of joint3: upper arm straight, forearm forward |

| Wave | Dual-arm routine (1.5x) | 67 dance |
|:---:|:---:|:---:|
| ![Wave](../docs/media/wave.gif) | ![Dual-arm routine](../docs/media/dual_arm.gif) | ![67 dance](../docs/media/dance67.gif) |

Every gesture starts and ends at the SRDF `home` pose, so gestures can be chained in any order.
The gestures are YAML files in [`motions/`](motions) (the motion library, see
[Adding a motion](#adding-a-motion)); `motion_demo` and the arm motion player both read them.
The original `arm_motion_node` (home → raised → home over the trajectory topics) is unchanged.

## Run
Start a simulation first, in a terminal with the same `ROS_DOMAIN_ID` as below:
```bash
source /opt/ros/jazzy/setup.bash
source ~/arm_body_ws/install/setup.bash
export ROS_DOMAIN_ID=215 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1
export GZ_PARTITION=moretea_215

ros2 launch arm_body_moveit_config moveit_gazebo.launch.py   # Gazebo + MoveIt + RViz
# or, without physics:
ros2 launch arm_body_moveit_config demo.launch.py            # mock hardware + MoveIt + RViz
```
Then, in a second terminal with the same exports:
```bash
ros2 run arm_motion_demos motion_demo                                        # wave, dual_arm, dance67
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[wave]"
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[wave]" -p wave_arm:=arm_b
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[dual_arm]"
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[dance67]" -p dance_tempo:=40.0 -p dance_amplitude:=0.3
ros2 run arm_motion_demos motion_demo --ros-args -p "sequence:=[home, wave, dual_arm, dance67, home]"
ros2 run arm_motion_demos motion_demo --ros-args -p dry_run:=true            # check only, no motion
```
Don't press **Plan & Execute** in RViz while a demo runs: both would command the same
controllers. motion_demo notices (the arms leave its plan) and stops, but avoid it.

## Tests
```bash
colcon test --packages-select arm_motion_demos && colcon test-result --verbose  # offline, no robot
ros2 run arm_motion_demos check_motion_demos     # live, against a running mock or Gazebo
```

## Adding a motion
Each `*.yaml` file in `motions/` (or in a `motion_folders` folder) is one motion; its `name`
is what goes in `sequence`. The format is documented at the top of
[`motion_library.py`](arm_motion_demos/motion_library.py); `motions/wave.yaml` is the
simplest example:
```yaml
name: wave
title: Wave
description: One arm out to the side, forearm up, hand swaying side to side.
parameters:
  repetitions: {type: int, default: 3, min: 1, max: 20, help: number of waves}
  arm: {type: choice, default: arm_a, choices: [arm_a, arm_b], help: which arm waves}
poses:                      # home plus these joints (rad); joint1..joint6 per arm
  hand_up:
    "{arm}": [1.3, -1.57, -1.2, -0.3, 1.57, 0.01]
    "{arm}_gripper_joint": 0.6
steps:
  - move: hand_up
    label: "{arm}: raise hand"
  - oscillate:
      amplitude: {"{arm}": [0.0, 0.0, 0.25, 0.2, 0.25, 0.0]}
      cycles: repetitions
    label: "{arm}: wave x{repetitions}"
  - move: home
    label: back to home
```
Steps are `move` (to a pose), `oscillate` (sway around the current pose), `hold` and
`repeat`. Numbers can be expressions of the parameters (`2 * 60 / tempo`). A motion must end
at home and pass every check above; a file that does not is skipped with the reason (and
`motion_demo` says so). Run `colcon build` after editing the files in `motions/`.
The YAML files reproduce the earlier Python gestures sample for sample
(`test/test_motion_library.py` compares them with a frozen copy, `test/legacy_gestures.py`).

## ArUco reach (arm A's gripper camera)
`aruco_reach` finds an ArUco tag (DICT_4X4_50, id 0, 50 mm black square by default) with the
OV3660 camera on arm A's gripper and moves the jaw tip `standoff` m in front of it:
1. moves arm A to `look_pose` (taught on the real arm, camera facing the work area);
2. averages the tag pose over `samples` frames (solvePnP) and transforms it to `world` with TF;
3. solves IK for the jaw tip (5-DOF arm: position exact, jaws turned towards the tag as far as possible);
4. plans and executes the joint goal with MoveIt (collision-checked).

The camera pose comes from the Fusion assembly "new mount (face up)": see
`arm_body_description/urdf/arm_a_camera.xacro` (`arm_a_camera_optical_frame`).

```bash
# simulated camera and tag (needs working Gazebo rendering)
ros2 launch arm_motion_demos aruco_reach_sim.launch.py
# or: real OV3660 (ESP32 MJPEG stream) + real tag, simulated arm
ros2 launch arm_motion_demos aruco_reach_sim.launch.py camera:=stream stream_url:=http://<esp32-ip>:81/stream

# second terminal
ros2 run arm_motion_demos aruco_reach
ros2 run arm_motion_demos aruco_reach --ros-args -p standoff:=0.15 -p dry_run:=true
```
With `camera:=stream`, the simulated arm treats the real camera as if it were on the gripper at
`look_pose`: hold or mount the camera facing forward, and put the tag 25–30 cm in front of it,
roughly level with it. A tag out of reach is refused (exit code 2), nothing moves.
`/arm_a/camera/aruco_debug` shows the detection (opened in rqt_image_view by the launch file).

| Parameter | Default | Meaning |
|---|---|---|
| `marker_id` / `marker_size` / `dictionary` | `0` / `0.05` / `DICT_4X4_50` | Tag to look for; size is the black square, in m |
| `standoff` | `0.0` | Jaw tip distance in front of the tag (m); 0 = touch the tag |
| `look_pose` | `[0.009, -3.019, 0.143, 0.982, 2.109, 2.022]` (SRDF `aruco_look`, taught on the real arm) | Arm A joints for detection; `[]` = detect from where it is |
| `samples` | `10` | Frames averaged |
| `max_position_error` | `0.01` | Refuse if IK can't get the jaw tip this close to the goal (m) |
| `settle_tolerance` | `0.05` | Per-joint error (rad) at which a move counts as arrived; real servos sag a little under load |
| `speed` | `0` | MoveIt velocity/acceleration scaling; `0` = the main speed (`default_*_scaling_factor` in `arm_body_moveit_config/config/joint_limits.yaml`), same as RViz's Plan & Execute |
| `dry_run` | `false` | Detect and plan only |
| `hold_roll` | `false` | Keep joint6 (wrist roll) where it is in the IK; for a real arm without a joint6 servo |

`camera_stream` parameters (launch arguments with `camera:=stream`): `stream_url` (MJPEG URL or
device index), `horizontal_fov` (rad, used without calibration; 0.96 ≈ a 66° diagonal lens),
`calibration_file` (ROS camera calibration YAML, e.g. from `camera_calibration`), `rotation`
(0/90/180/270° clockwise, to turn a sideways or upside-down camera image upright), `fourcc`
(device index only: pixel format to request, e.g. `MJPG` when the default raw format is slow).
Without a calibration, distances can be ~10 % off: calibrate before using it on the real arm.
