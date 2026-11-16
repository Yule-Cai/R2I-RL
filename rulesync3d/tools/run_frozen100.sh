#!/bin/bash
# Frozen 100-case feasibility campaign: 20 scenarios x 5 seeds.
# Manifest (reports/RULESYNC_3D_MANIFEST.json) is frozen BEFORE launch.
# Each seed = full 4-world campaign with shifted epoch blocks + suffixed
# outputs. ~3.5 h wall. No cherry-picking: all rows count.
export SEED=0
SCRIPT_DIR=$(dirname "$0")
echo "[frozen100] manifest must already be frozen; starting seeds 0-4"
for K in 0 1 2 3 4; do
  echo "[frozen100] ===== SEED $K ====="
  SEED=$K bash $SCRIPT_DIR/run_stage_a.sh || echo "[frozen100] seed $K exit: $?"
done
echo "[frozen100] all seeds done."
