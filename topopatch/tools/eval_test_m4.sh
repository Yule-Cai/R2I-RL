#!/bin/bash
# M4 FINAL TEST evaluation: gated, tau=-inf (full records), frozen hashes.
# Per-method VAL operating points applied in analysis (exact recompute).
# Usage: eval_test_m4.sh
set -u
export ROS_DOMAIN_ID=7
PY=~/miniforge3/envs/causal-repair/bin/python
CK=~/R2I-RL-checkout
M1=~/rulepatch/runs/m1/best.pt
M3U=~/rulepatch/runs/m3/best.pt
NOTOPO=~/rulepatch/topo_notopo/best.pt
FULL=~/rulepatch/topo_full/best.pt
SCOPE=~/rulepatch/topo_scope/best.pt
FR=~/rulepatch/freeze_m4.json
BND="-10 -10 9.2 9.2"

run_one() { # <tag> <map_id> <ext_dir> <raw_manifest>
  $PY -m topopatch.topo.evaluate_topo \
    --data "$3" --map-id "$2" --split test \
    --pgm ~/rulepatch/topo_maps/"$2".pgm --res-in 0.05 --ox -10 --oy -10 \
    --bounds $BND \
    --m1 $M1 --m3unstruct $M3U --notopo $NOTOPO --full $FULL \
    --scope $SCOPE --freeze $FR \
    --rollout --records ~/rulepatch/m4test_records_"$1".jsonl \
    --manifest "$4" --out ~/rulepatch/m4test_eval_"$1"
}
cd "$CK" || exit 1
run_one mazeB maze-B-s202 ~/rulepatch/topo_ext_maze-B-s202 \
  ~/rulepatch/topo_raw_maze-B-s202/manifest.jsonl
run_one narrowB narrow-B-s302 ~/rulepatch/topo_ext_narrow-B-s302 \
  ~/rulepatch/topo_raw_narrow-B-s302/manifest.jsonl
echo "[eval-test-m4] ALL DONE"
