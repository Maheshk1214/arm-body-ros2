#!/usr/bin/env bash
# GIP Garage Motion Player: one command to start the simulation and the menu.
#
#   ./run.sh              Gazebo window + menu
#   ./run.sh --headless   no Gazebo window (videos can still be recorded)
#   ./run.sh --attach     use a simulation that is already running on the same domain
#
# Settings: config/player.yaml next to this script (no rebuild needed after editing).
# (No 'set -u': ROS's setup.bash scripts use unset variables.)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fail() { echo "ERROR: $*" >&2; exit 1; }

# The ROS workspace is the first folder above this script that has install/setup.bash
WS="$HERE"
while [ "$WS" != "/" ] && [ ! -f "$WS/install/setup.bash" ]; do WS="$(dirname "$WS")"; done
[ -f "$WS/install/setup.bash" ] || fail "no built ROS workspace found above $HERE.
  Build it first:  cd <your_ws> && colcon build && source install/setup.bash"

ROS_SETUP=/opt/ros/jazzy/setup.bash
[ -f "$ROS_SETUP" ] || fail "ROS 2 Jazzy not found ($ROS_SETUP). See docs/RUN_GUIDE.md section 1."

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

for pkg in garage_motion_player arm_motion_demos arm_body_moveit_config arm_body_description; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fail "package '$pkg' is not built.
  Run:  cd $WS && colcon build && source install/setup.bash"
done
for cmd in gz ffmpeg; do
  command -v "$cmd" >/dev/null || fail "'$cmd' is not installed (see the README's Requirements)."
done

CONFIG="$HERE/config/player.yaml"
[ -f "$CONFIG" ] || fail "settings file missing: $CONFIG"

exec ros2 run garage_motion_player player --config "$CONFIG" --workspace "$WS" "$@"
