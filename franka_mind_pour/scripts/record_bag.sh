#!/bin/bash
# `rosbag record` wrapper used by mind_pour.launch, which cannot create the
# output directory itself (and rosbag refuses to write into a missing one).
#
# Usage: record_bag.sh <output_dir> <prefix> <topic> [topic ...]
# Bags are named <output_dir>/<prefix>_<date>_<time>.bag.
#
# exec, so rosbag receives roslaunch's SIGINT directly and closes the bag
# cleanly on shutdown.

if [ $# -lt 3 ]; then
  echo "Usage: record_bag.sh <output_dir> <prefix> <topic> [topic ...]"
  exit 1
fi

OUTPUT_DIR=$1
PREFIX=$2
shift 2

if ! mkdir -p "$OUTPUT_DIR"; then
  echo "record_bag.sh: could not create $OUTPUT_DIR, not recording"
  exit 1
fi

echo "record_bag.sh: recording $* into $OUTPUT_DIR"
exec rosbag record --output-prefix "$OUTPUT_DIR/$PREFIX" "$@"
