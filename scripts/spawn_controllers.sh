#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="${1:?controllers config file required}"
WAIT_TIMEOUT="${2:-180}"

export ROS2CLI_USE_DAEMON=0

echo "[controller_bootstrap] waiting for controller manager service..."

controller_manager_path=""
deadline=$((SECONDS + WAIT_TIMEOUT))

while (( SECONDS < deadline )); do
  for candidate in /controller_manager /panda/controller_manager; do
    if timeout 1s ros2 service list 2>/dev/null | grep -q "^${candidate}/list_controllers$"; then
      # Guard against stale discovery entries from recently stopped simulations.
      if timeout 2s ros2 control list_controllers --controller-manager "$candidate" >/dev/null 2>&1; then
        controller_manager_path="$candidate"
        break 2
      fi
    fi
  done
  sleep 1
done

if [[ -z "$controller_manager_path" ]]; then
  echo "[controller_bootstrap] ERROR: no controller manager detected within ${WAIT_TIMEOUT}s."
  echo "[controller_bootstrap] Tried: /controller_manager and /panda/controller_manager"
  echo "[controller_bootstrap] Available services snapshot:"
  ros2 service list | sort || true
  exit 1
fi

echo "[controller_bootstrap] using ${controller_manager_path}"

spawn_controller() {
  local controller_name="$1"

  if ros2 run controller_manager spawner \
    "${controller_name}" \
    --controller-manager "${controller_manager_path}" \
    --controller-manager-timeout "${WAIT_TIMEOUT}" \
    --switch-timeout "${WAIT_TIMEOUT}" \
    --service-call-timeout 10.0 >/dev/null 2>&1; then
    echo "[controller_bootstrap] ${controller_name} activated"
    return 0
  fi

  echo "[controller_bootstrap] ERROR: failed to activate ${controller_name}"
  echo "[controller_bootstrap] controller snapshot:"
  ros2 control list_controllers --controller-manager "${controller_manager_path}" || true
  return 1
}

spawn_controller joint_state_broadcaster
spawn_controller panda_arm_controller

echo "[controller_bootstrap] controllers are active"
