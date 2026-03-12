#!/usr/bin/env bash
set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass_count=0
fail_count=0

pass() {
  echo -e "${GREEN}[PASS]${NC} $1"
  pass_count=$((pass_count + 1))
}

fail() {
  echo -e "${RED}[FAIL]${NC} $1"
  fail_count=$((fail_count + 1))
}

warn() {
  echo -e "${YELLOW}[WARN]${NC} $1"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    fail "Missing command: $1"
    exit 1
  fi
}

get_rate_hz() {
  local topic=$1
  timeout 6s ros2 topic hz "$topic" 2>/dev/null |
    awk '/average rate:/ {print $3; exit}'
}

topic_has_message() {
  local topic=$1
  timeout 8s ros2 topic echo "$topic" --once >/dev/null 2>&1
}

check_motion_delta() {
  local max_seen=0
  local attempt
  for attempt in 1 2 3; do
    local f1 f2 delta
    f1=$(mktemp)
    f2=$(mktemp)
    timeout 5s ros2 topic echo /joint_states --once >"$f1" || true
    sleep 1
    timeout 5s ros2 topic echo /joint_states --once >"$f2" || true

    delta=$(python3 - "$f1" "$f2" <<'PY'
import re
import sys


def parse_positions(path):
    text = open(path, 'r', encoding='utf-8').read()
    match = re.search(r'position:\n((?:- .*\n)+)', text)
    if not match:
        print('0.0')
        raise SystemExit(0)
    return [float(line.strip()[2:]) for line in match.group(1).strip().splitlines()]

p1 = parse_positions(sys.argv[1])
p2 = parse_positions(sys.argv[2])
delta = [abs(b - a) for a, b in zip(p1, p2)]
print(max(delta))
PY
)

    rm -f "$f1" "$f2"

    if [[ -n "$delta" ]] && awk -v cur="$delta" -v best="$max_seen" 'BEGIN {exit (cur > best ? 0 : 1)}'; then
      max_seen="$delta"
    fi

    if awk -v v="$max_seen" 'BEGIN {exit (v >= 0.05 ? 0 : 1)}'; then
      echo "$max_seen"
      return 0
    fi
  done

  echo "$max_seen"
}

source_if_exists() {
  local setup_file=$1
  if [[ -f "$setup_file" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

main() {
  set -u
  require_cmd ros2
  require_cmd timeout
  require_cmd python3

  source_if_exists /opt/ros/humble/setup.bash
  source_if_exists /home/zoel/ruckig_ws/install/setup.bash

  echo "== OTG Planner Simulation Health Check =="

  local nodes
  nodes=$(timeout 5s ros2 node list 2>/dev/null || true)
  if [[ -z "$nodes" ]]; then
    fail "No ROS nodes found. Did you start: ros2 launch otg_planner simulation.launch.py ?"
    exit 1
  fi

  for required_node in /controller_manager /otg_planner_node /robot_state_publisher /ros_gz_bridge; do
    if echo "$nodes" | grep -Eq "^${required_node}$"; then
      pass "Node alive: ${required_node}"
    else
      fail "Node missing: ${required_node}"
    fi
  done

  local controllers
  controllers=$(timeout 6s ros2 control list_controllers 2>/dev/null || true)
  if echo "$controllers" | grep -Eq 'joint_state_broadcaster.*active'; then
    pass 'Controller active: joint_state_broadcaster'
  else
    fail 'Controller not active: joint_state_broadcaster'
  fi

  if echo "$controllers" | grep -Eq 'panda_arm_controller.*active'; then
    pass 'Controller active: panda_arm_controller'
  else
    fail 'Controller not active: panda_arm_controller'
  fi

  if timeout 5s ros2 topic echo /clock --once >/dev/null 2>&1; then
    pass '/clock is publishing (sim time running)'
  else
    fail '/clock not receiving messages'
  fi

  if timeout 5s ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    pass '/joint_states is publishing'
  else
    fail '/joint_states has no messages'
  fi

  local hz_joint
  hz_joint=$(get_rate_hz /joint_states || true)

  if [[ -n "$hz_joint" ]]; then
    if awk -v v="$hz_joint" 'BEGIN {exit (v >= 20.0 ? 0 : 1)}'; then
      pass "/joint_states rate looks good: ${hz_joint} Hz"
    else
      fail "/joint_states rate too low: ${hz_joint} Hz"
    fi
  else
    fail 'Could not measure /joint_states rate'
  fi

  if topic_has_message /panda_arm_controller/joint_trajectory; then
    pass '/panda_arm_controller/joint_trajectory is publishing'
  else
    fail '/panda_arm_controller/joint_trajectory has no messages'
  fi

  if topic_has_message /otg/planned_trajectory; then
    pass '/otg/planned_trajectory is publishing'
  else
    fail '/otg/planned_trajectory has no messages'
  fi

  local max_delta
  max_delta=$(check_motion_delta || true)
  if [[ -n "$max_delta" ]] && awk -v v="$max_delta" 'BEGIN {exit (v >= 0.05 ? 0 : 1)}'; then
    pass "Joint motion detected (max |delta| over 1s = ${max_delta} rad)"
  else
    fail "No significant joint motion detected (max |delta| over 1s = ${max_delta:-N/A})"
    warn 'If Gazebo looks frozen, check GUI Play/Pause button and duplicate launch processes.'
  fi

  echo
  if [[ $fail_count -eq 0 ]]; then
    echo -e "${GREEN}Health check passed (${pass_count} checks).${NC}"
  else
    echo -e "${RED}Health check failed: ${fail_count} failed, ${pass_count} passed.${NC}"
    exit 2
  fi
}

main "$@"
