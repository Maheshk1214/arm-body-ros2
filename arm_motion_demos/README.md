# arm_motion_demos

Gesture demonstrations for the dual-arm robot, in simulation (MoveIt mock demo or Gazebo):

| Gesture | What it does |
|---|---|
| `wave` | One arm swings out to the side, forearm up, gripper open; the hand sways ~20 cm side to side |
| `dual_arm` | Both arms forward → raised together → alternating up/down → out to the sides → symmetric flaps |
| `dance67` | "6-7": forearms forward, palms (grippers) open, hands bobbing up and down in turn; the wrists counter-rotate so the palms stay level |

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
source ~/moretea_arm_body_ws/install/setup.bash
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

