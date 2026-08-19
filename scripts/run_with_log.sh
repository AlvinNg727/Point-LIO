#!/usr/bin/env bash
# Streams a roslaunch node's stdout+stderr to both the console and a log file.
# Uses stdbuf to line-buffer output, so messages survive a crash/segfault
# (buffered output is normally lost when the node dies).
# Usage:
#   <node launch-prefix="$(find point_lio)/scripts/run_with_log.sh <log_file> ..." />
set -o pipefail

LOG_FILE="${1:?usage: run_with_log.sh <log_file> [node_command...]}"
shift

{
    echo "==== $(date '+%Y-%m-%d %H:%M:%S') node start: $* ===="
    exec stdbuf -oL -eL "$@"
} 2>&1 | tee -a "$LOG_FILE"