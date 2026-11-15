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
# full = a real session. udp, move and rotate each run one step on its own, for
# debugging; see usage below.
STAGE=full
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
  echo "  participant:=<id>             required (debug stages: default debug)"
  echo "  session:=<n>                  required (debug stages: default 0)"
  echo "  stage:=full|udp|move|rotate   default ${STAGE}. The others debug one step each:"
  echo "                                  udp     NO ROBOT. Only the UDP bridge, printing the"
  echo "                                          rate of the commands it receives."
  echo "                                  move    only the move to the start pose, then exit."
  echo "                                  rotate  no move: hold the arm where it is and rotate"
  echo "                                          joint 7 on commands. No results row."
  echo "  robot_ip:=<ip>                default ${ROBOT_IP}"
  echo "  record:=true|false            record the session into a rosbag, default ${RECORD}"
  echo "  record_dir:=<dir>             where the bags go, default ${RECORD_DIR}"
  echo "  results_file:=<csv>           default <record_dir>/results.csv"
  echo ""
  echo "Examples:"
  echo "  ./scripts/run_session.sh participant:=P01 session:=1"
  echo "  ./scripts/run_session.sh participant:=P01 session:=2 record_dir:=/data/fmp"
  echo "  ./scripts/run_session.sh stage:=udp"
  echo ""
  echo "Simulated commands, from this PC (any stage but move):"
  echo "  python3 scripts/fmp_udp_sender.py --host 127.0.0.1 --pattern=1:1    # 1 s clockwise"
  echo "  python3 scripts/fmp_udp_sender.py --host 127.0.0.1 --pattern=-1:1   # 1 s anti-clockwise"
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
    stage:=*) STAGE=${arg#*:=} ;;
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

case "$STAGE" in
  full) ;;
  udp | move | rotate)
    PARTICIPANT=${PARTICIPANT:-debug}
    SESSION=${SESSION:-0}
    ;;
  *)
    echo "stage must be full, udp, move or rotate, got '${STAGE}'"
    usage
    exit 1
    ;;
esac
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

echo "Participant ${PARTICIPANT}, session ${SESSION}, stage ${STAGE}, robot ip: ${ROBOT_IP}"
case "$STAGE" in
  full)
    echo "The arm MOVES ON ITS OWN to the start pose first. Keep its path clear."
    echo "It then rotates joint 7 on the EEG commands until it is back at its start"
    echo "angle; the trial time is appended to ${RESULTS_FILE}."
    ;;
  udp)
    echo "DEBUG: UDP bridge only, the robot is not used. Ctrl+C to stop."
    ;;
  move)
    echo "DEBUG: the arm MOVES ON ITS OWN to the start pose, then everything exits."
    echo "Keep its path clear."
    ;;
  rotate)
    echo "DEBUG: the arm stays where it is and joint 7 rotates on the commands until it"
    echo "is back at its start angle (or Ctrl+C). No results row is written."
    ;;
esac
if [ "$RECORD" = "true" ] && [ "$STAGE" != "udp" ]; then
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

if [ "$STAGE" = "udp" ]; then
  roslaunch franka_mind_pour debug_udp.launch &
else
  roslaunch franka_mind_pour mind_pour.launch \
    participant:=$PARTICIPANT \
    session:=$SESSION \
    stage:=$STAGE \
    robot_ip:=$ROBOT_IP \
    record:=$RECORD \
    record_dir:=$RECORD_DIR \
    results_file:=$RESULTS_FILE &
fi
FMP_PID=$!

# The launch file ends by itself when the session is complete.
wait $FMP_PID
# Remove ANSI escape sequences from ROS log files
find ~/.ros/log/latest/ -type f -name "*.log" -exec sed -i 's/\x1b\[[0-9;]*m//g' {} \;
