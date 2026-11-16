#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname "$0")" && pwd)"
REPO_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
FORMAL_DIR="$REPO_ROOT/experiments/rulesync_experiments/formal_dualmap_20261108_v2"
DIAGNOSTIC_DIR="$REPO_ROOT/experiments/rulesync_experiments/authorization_diagnostic_20261115"
export PYTHONPATH="$FORMAL_DIR:$DIAGNOSTIC_DIR:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON_BIN" -m pytest -q "$FORMAL_DIR/test_contract_v1.py" "$DIAGNOSTIC_DIR/test_observer_ledger.py"
