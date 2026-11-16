#!/bin/bash
# RulePatchNet M2 remote pipeline (onexplayer): extract -> train x3 -> eval -> replay.
# Usage: bash run_pipeline.sh  (logs to ~/rulepatch/logs/pipeline.log)
set -u
RP=/home/yule/rulepatch
PY=/home/yule/miniforge3/envs/causal-repair/bin/python
export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:${PYTHONPATH:-}
export OMP_NUM_THREADS=4
mkdir -p $RP/extracted $RP/runs $RP/eval $RP/logs
log() { echo "[pipe] $(date '+%H:%M:%S') $*" | tee -a $RP/logs/pipeline.log; }

log "extract start"
$PY $RP/tools/run_extract.py --data $RP/data \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --out $RP/extracted 2>&1 | tail -30 | tee -a $RP/logs/pipeline.log
test -f $RP/extracted/train_0000.npz || { log "extract failed"; exit 1; }

for CFG in "m3 3" "m1 1"; do
  set -- $CFG; NAME=$1; M=$2
  log "train $NAME start"
  $PY -m rp.train --data $RP/extracted --out $RP/runs/$NAME \
    --m-hyp $M --epochs 80 --batch 64 --workers 2 \
    2>&1 | tail -15 | tee -a $RP/logs/pipeline.log
done
log "train m3norule start"
$PY -m rp.train --data $RP/extracted --out $RP/runs/m3norule \
  --m-hyp 3 --zero-rule --epochs 80 --batch 64 --workers 2 \
  2>&1 | tail -15 | tee -a $RP/logs/pipeline.log

log "eval start"
$PY -m rp.evaluate --data $RP/extracted \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --m3 $RP/runs/m3/best.pt --m1 $RP/runs/m1/best.pt \
  --m3norule $RP/runs/m3norule/best.pt \
  --out $RP/eval --rollout 2>&1 | tail -60 | tee -a $RP/logs/pipeline.log

log "replay scan start"
$PY $RP/tools/replay_sample.py --data $RP/extracted \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --weights $RP/runs/m3/best_fused.npz \
  --out $RP/eval/replay_h1fail_h2ok.json \
  2>&1 | tail -30 | tee -a $RP/logs/pipeline.log
log "pipeline done"
