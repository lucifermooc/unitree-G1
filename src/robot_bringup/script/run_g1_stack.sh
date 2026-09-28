#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_humble_env.sh"

exec ros2 launch robot_bringup robot.launch.py "$@"
