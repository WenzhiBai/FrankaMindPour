#!/bin/bash

# Source environment. ROS's setup.sh forwards whatever positional arguments are
# set to _setup_util.py, so stash ours over the sourcing and put them back.
SCRIPT_ARGS=("$@")
set --
source /opt/ros/noetic/setup.bash
source ~/workspace/catkin_ws_franka/devel/setup.bash
source ~/workspace/catkin_ws_fmp/devel/setup.bash
set -- "${SCRIPT_ARGS[@]}"

# Defaults. Override any of them on the command line, roslaunch style, in any
# order: ./scripts/run_session.sh participant:=P01 session:=3
PARTICIPANT=
SESSION=
ROBOT_IP=192.168.1.11
RECORD=true
# One bag per session, named mind_pour_<participant>_s<session>_<date>_<time>.bag.
RECORD_DIR=$HOME/fmp_logs
# One row per completed session is appended here.
RESULTS_FILE=

usage() {
  echo "Usage: ./scripts/run_session.sh participant:=<id> session:=<n> [key:=value ...]"
  echo "Runs one session: move to the start pose, rotate joint 7 on EEG commands, and stop"
  echo "once joint 7 is back at its start angle. Arguments, in any order:"
  echo "  participant:=<id>             required"
  echo "  session:=<n>                  required"
  echo "  robot_ip:=<ip>                default ${ROBOT_IP}"
  echo "  record:=true|false            record the session into a rosbag, default ${RECORD}"
  echo "  record_dir:=<dir>             where the bags go, default ${RECORD_DIR}"
  echo "  results_file:=<csv>           default <record_dir>/results.csv"
  echo ""
  echo "Examples:"
  echo "  ./scripts/run_session.sh participant:=P01 session:=1"
  echo "  ./scripts/run_session.sh participant:=P01 session:=2 record_dir:=/data/fmp"
  echo ""
  echo "Everything else (start pose, speed, limits, UDP port) is in"
  echo "franka_mind_pour/config/mind_pour.yaml."
}

# Only the keys above are accepted: roslaunch ignores arguments a launch file
# does not declare, so a typo would otherwise silently leave a setting at its
# default.
for arg in "$@"; do
  case "$arg" in
    participant:=*) PARTICIPANT=${arg#*:=} ;;
    session:=*) SESSION=${arg#*:=} ;;
    robot_ip:=*) ROBOT_IP=${arg#*:=} ;;
    record:=*) RECORD=${arg#*:=} ;;
    record_dir:=*) RECORD_DIR=${arg#*:=} ;;
    results_file:=*) RESULTS_FILE=${arg#*:=} ;;
    -h | --help | help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument '${arg}'."
      usage
      exit 1
      ;;
  esac
done

if [ -z "$PARTICIPANT" ] || [ -z "$SESSION" ]; then
  echo "participant and session are required."
  usage
  exit 1
fi
case "$RECORD" in
  true | false) ;;
  *)
    echo "record must be true or false, got '${RECORD}'"
    usage
    exit 1
    ;;
esac
RESULTS_FILE=${RESULTS_FILE:-$RECORD_DIR/results.csv}

echo "Participant ${PARTICIPANT}, session ${SESSION}, robot ip: ${ROBOT_IP}"
echo "The arm MOVES ON ITS OWN to the start pose first. Keep its path clear."
echo "It then rotates joint 7 on the EEG commands until it is back at its start"
echo "angle; the trial time is appended to ${RESULTS_FILE}."
if [ "$RECORD" = "true" ]; then
  echo "Recording commands and controller state into a rosbag under ${RECORD_DIR}."
fi

# Trap Ctrl+C
trap ctrl_c INT
ctrl_c() {
  echo "Stopping child processes..."
  # SIGINT, so roslaunch shuts its nodes down the same way it would on Ctrl+C in
  # its own terminal and `rosbag record` closes the bag properly.
  kill -INT $FMP_PID 2>/dev/null
  wait $FMP_PID 2>/dev/null
}

roslaunch franka_mind_pour mind_pour.launch \
  participant:=$PARTICIPANT \
  session:=$SESSION \
  robot_ip:=$ROBOT_IP \
  record:=$RECORD \
  record_dir:=$RECORD_DIR \
  results_file:=$RESULTS_FILE &
FMP_PID=$!

# The launch file ends by itself when the session is complete.
wait $FMP_PID
# Remove ANSI escape sequences from ROS log files
find ~/.ros/log/latest/ -type f -name "*.log" -exec sed -i 's/\x1b\[[0-9;]*m//g' {} \;
