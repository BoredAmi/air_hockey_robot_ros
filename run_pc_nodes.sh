#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WITH_CONFIG_TUNER="${WITH_CONFIG_TUNER:-1}"

mkdir -p "${LOG_DIR}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

pids=()

start_node() {
  local node_name="$1"
  local log_file="${LOG_DIR}/${node_name}.log"
  shift
  echo "Starting ${node_name} ..."
  "$@" >"${log_file}" 2>&1 &
  pids+=("$!")
  echo "  log: ${log_file}"
}

cleanup() {
  echo
  echo "Stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}

trap cleanup EXIT INT TERM

start_node "perception_node" ros2 run pc_node perception_node
start_node "trajectory_node" ros2 run pc_node trajectory_node
start_node "movement_node" ros2 run pc_node movement_node

if [[ "${WITH_CONFIG_TUNER}" == "10" ]]; then
  start_node "config_tuner" ros2 run pc_node config_tuner
fi

echo
 echo "All requested nodes started."
 echo "Logs are in: ${LOG_DIR}"
 echo "Press Ctrl+C to stop all nodes."

wait
