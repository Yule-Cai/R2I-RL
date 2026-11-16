#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

export PYTHONPATH="$REPO_ROOT:$REPO_ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

"$PYTHON_BIN" -m pytest -q \
  "$REPO_ROOT/rulesync3d/tests" \
  "$REPO_ROOT/rulesync/src/rulesync_manager/test" \
  "$REPO_ROOT"/scripts/test_*.py

"$PYTHON_BIN" -m py_compile \
  "$REPO_ROOT/rulesync3d/rulesync3d/gate2d.py" \
  "$REPO_ROOT/rulesync3d/rulesync3d/ruleguard.py" \
  "$REPO_ROOT/rulesync3d/rulesync3d/authorization.py" \
  "$REPO_ROOT/rulesync/src/rulesync_guard/rulesync_guard/guard_node.py" \
  "$REPO_ROOT/topopatch_ros/topopatch_ros/gate_node.py" \
  "$REPO_ROOT/topopatch_ros/topopatch_ros/uav_case_driver.py" \
  "$REPO_ROOT/topopatch_ros/topopatch_ros/topo_case_driver.py" \
  "$REPO_ROOT/rulesync3d/tools/px4_gated_mission.py"
