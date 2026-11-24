#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"

GUI_FLAG="-s"
if [[ "${1:-}" == "--gui" ]]; then
  GUI_FLAG=""
fi

if ! command -v ign >/dev/null 2>&1; then
  echo "[CRAFT] Gazebo Fortress ('ign') is not installed. Run: bash ${SCRIPT_DIR}/setup_gazebo_fortress.sh" >&2
  exit 1
fi

echo "[CRAFT] Starting Gazebo Fortress world: ${SCRIPT_DIR}/worlds/craft_cafe_fortress.sdf"
ign gazebo ${GUI_FLAG} -r "${SCRIPT_DIR}/worlds/craft_cafe_fortress.sdf" &
GZ_PID=$!

BRIDGE_PID=""
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "[CRAFT] Starting ROS 2 Humble ros_gz_bridge..."
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${SCRIPT_DIR}/config/ros_gz_bridge.yaml" &
    BRIDGE_PID=$!
  fi
fi

trap 'kill ${GZ_PID} ${BRIDGE_PID:-} 2>/dev/null || true' EXIT INT TERM
wait "${GZ_PID}"
