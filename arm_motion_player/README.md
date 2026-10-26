# Moretea Arm Motion Player

Play robot movements in the Gazebo simulation from a simple menu: no ROS 2 commands needed.

```
==============================================
          MORETEA ARM MOTION PLAYER
==============================================
 Robot: Left arm (arm_a)   Speed: 0.50   Pose: at home
----------------------------------------------
 SELECT ROBOT   [1] Left arm  [2] Right arm  [3] Both arms
 ACTIONS        [D] Move one joint (DOF)
                [J] Live control: hold keys to move joints
 MOTIONS        [M] Play a motion: dance67, dual_arm, wave
 OTHER          [H] Return to home
                [V] Change speed
                [Q] Quit
 While the robot moves: type S + Enter (or Ctrl-C) to stop.
==============================================
```

## Start
```bash
cd ~/moretea_arm_body_ws/src/arm-body-ros2/arm_motion_player
./run.sh               # Gazebo window + menu
./run.sh --headless    # no Gazebo window
```
`run.sh` checks the setup, starts Gazebo with the robot, its controllers and MoveIt, waits until all
of them are ready, and opens the menu. **Q** (or Ctrl-C at the menu) offers to bring the robot home
and then shuts the simulation down. Starting takes about a minute.

It runs on its own ROS domain (`ros_domain_id` in `config/player.yaml`, default 215) and refuses to
start if something else uses that domain. To use a simulation you started yourself on that
domain: `./run.sh --attach`.

## Using the menu
- **1 / 2 / 3** choose the left arm (`arm_a`, the robot's own left: on your right when you face
  it), the right arm (`arm_b`) or both.
- **D** lists the joints with their range and current angle; pick one. Only that joint moves,
  towards each of its limits (as far as its own range allows) and back. With **Both arms**, the same
  joint moves on both arms.
- **J** is live control: hold a key and the joint moves, let go and it stops. No Enter needed.

  | Keys | Joint |   | Keys | Joint |
  |---|---|---|---|---|
  | `Q` / `A` | joint1 up / down | | `T` / `G` | joint5 |
  | `W` / `S` | joint2 | | `Y` / `H` | joint6 |
  | `E` / `D` | joint3 | | `U` / `J` | gripper |
  | `R` / `F` | joint4 | | | |

  `[` / `]` slower / faster (0.05 … 0.5 rad/s, starts at 0.2), **SPACE** stops at once,
  **X** or **Esc** goes back to the menu, Ctrl-C stops too. With **Both arms** a key moves that
  joint on both arms. The bottom line shows every joint's angle live.

  How it stays safe: the joint speeds up and slows down smoothly, brakes in time to stop 0.01
  rad inside its limit, and before every command MoveIt checks the pose where the joint would
  stop if it braked now. If that pose collides, the joint stops before it and that direction is
  blocked until you move the other way. If the robot does not follow (more than 0.2 rad off) or
  the joint states stop, it holds and live control ends.

  A terminal does not report when a key is let go, only the key's repeats while it is held, so
  the player counts a key as released when its repeats stop: 0.15 s after the last one, or 0.6
  s after a single tap (your keyboard waits 500 ms before repeating). A tap therefore moves a
  joint a few degrees. If your keyboard's repeat delay is longer, raise
  `live_control.first_gap` in `config/player.yaml` (`xset q` shows the delay).
  Live control needs a real terminal; it is not available when input comes from a pipe.
- **M** lists the motion library (wave, dual_arm, the 67 dance, and any motion files you add)
  with each motion's settings and their ranges. Pick one by number or name, then press Enter
  for the defaults or type changes, e.g. `repetitions=4 tempo=40`. With the left or right
  arm selected, the wave uses that arm. The robot goes home first if it is not there.
- **H** returns home. **V** changes the speed (0.1 slow … 1.0 fastest allowed).
- **S + Enter** (or Ctrl-C) stops a motion; the robot holds where it is. Press **H** afterwards.

From home the arms hang about 2 cm above the floor, and several joints could only swing into it,
so **D** first lifts both arms to a *DOF demo pose* (arms forward and angled outward).

## Safety checks
Every motion, before it starts:
1. stays at least 0.01 rad inside each joint's own URDF limits (a joint held exactly on a limit
   jams in Gazebo), within the URDF velocity and MoveIt acceleration limits;
2. is collision-checked with MoveIt every 0.1 s (self-collision, the other arm, Gazebo's floor);
   a direction of a joint swing that would collide is skipped and reported;
3. starts only if the robot is not moving and is where the plan starts.

While moving, the player compares the joint angles the robot reports with the plan and stops the
robot if a joint is more than 0.2 rad off, gets within 3 mrad of a limit, or the joint states stop.
Success means the robot reported the final pose, not that a controller said "done".
Home uses the direct path when MoveIt finds it free, otherwise a MoveIt collision-aware plan.

## Adding your own motions
Put a `*.yaml` motion file in `~/moretea_arm_body_ws/motions/` (the folder in
`motions.folders` of `config/player.yaml`) and restart `./run.sh`: it appears under **M**. The format, with an example, is in
[`arm_motion_demos/README.md`](../arm_motion_demos/README.md#adding-a-motion). At start-up the
player builds every motion with its defaults and checks it against the joint limits; a file
that fails is left out and the reason is printed (and shown under **M**).

## Settings (`config/player.yaml`)
Edit and restart `./run.sh`; no rebuild needed. Main settings: `ros_domain_id`, `simulation.gui`,
`motion.speed`, the single-joint swing per joint (`joints.<name>.amplitude`, rad) and the DOF demo
pose (`dof_pose`).

## Tests
```bash
colcon test --packages-select arm_motion_player && colcon test-result --verbose   # no robot
ros2 run arm_motion_player check_player --run-sh "$PWD/run.sh"   # live: starts run.sh itself
```

## Troubleshooting
| Message | What to do |
|---|---|
| `package '…' is not built` | `cd ~/moretea_arm_body_ws && colcon build`, then run `./run.sh` again |
| `ROS domain 215 is already in use by …` | Close that simulation, or `./run.sh --attach` to use it, or change `ros_domain_id` |
| `not ready after … s, still waiting for …` | See the log file named at start-up (`logs/arm_motion_player/`) |
| `Not moved: … collision …` | The motion would hit something; it was not started |
| `Motion failed and was stopped` | The robot did not follow the plan; it holds. Press **H** |
