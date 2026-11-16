#!/bin/bash
# VAL evaluation over the 4 train maps (one occ grid per run).
# Usage: eval_val.sh  (reads ckpts + data from fixed ~/rulepatch paths)
set -u
export ROS_DOMAIN_ID=7
PY=~/miniforge3/envs/causal-repair/bin/python
CK=~/R2I-RL-checkout
M1=~/rulepatch/runs/m1/best.pt
M3U=~/rulepatch/runs/m3/best.pt
NOTOPO=~/rulepatch/topo_notopo/best.pt
FULL=~/rulepatch/topo_full/best.pt
SCOPE=~/rulepatch/topo_scope/best.pt
BND="-10 -10 9.2 9.2"

run_one() { # <tag> <map_id> <ext_dir> <raw_manifest>
  $PY -m topopatch.topo.evaluate_topo \
    --data "$3" --map-id "$2" --split val \
    --pgm ~/rulepatch/topo_maps/"$2".pgm --res-in 0.05 --ox -10 --oy -10 \
    --bounds $BND \
    --m1 $M1 --m3unstruct $M3U --notopo $NOTOPO --full $FULL --scope $SCOPE \
    --rollout --records ~/rulepatch/m4val_records_"$1".jsonl \
    --manifest "$4" --out ~/rulepatch/m4val_eval_"$1"
}
cd "$CK" || exit 1
run_one officeA office-A-s101 ~/rulepatch/topo_officeA_ext \
  ~/rulepatch/topo_officeA/manifest.jsonl
run_one officeB office-B-s102 ~/rulepatch/topo_ext_office-B-s102 \
  ~/rulepatch/topo_raw_office-B-s102/manifest.jsonl
run_one mazeA maze-A-s201 ~/rulepatch/topo_ext_maze-A-s201 \
  ~/rulepatch/topo_raw_maze-A-s201/manifest.jsonl
run_one narrowA narrow-A-s301 ~/rulepatch/topo_ext_narrow-A-s301 \
  ~/rulepatch/topo_raw_narrow-A-s301/manifest.jsonl
echo "[eval-val] ALL DONE"
