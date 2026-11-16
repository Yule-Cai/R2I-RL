#!/bin/bash
# Train all three configs + eval + replay (extraction already done).
# Usage: bash run_train_eval.sh (logs to ~/rulepatch/logs/train_eval.log)
set -u
RP=/home/yule/rulepatch
PY=/home/yule/miniforge3/envs/causal-repair/bin/python
export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:${PYTHONPATH:-}
export OMP_NUM_THREADS=4
mkdir -p $RP/runs $RP/eval
log() { echo "[te] $(date '+%H:%M:%S') $*" | tee -a $RP/logs/train_eval.log; }

log "train m3 start"
$PY -m rp.train --data $RP/extracted --out $RP/runs/m3 \
  --m-hyp 3 --epochs 200 --batch 64 --workers 0 --margin-lambda 3 --margin 0.0625 \
  2>&1 | tail -8 | tee -a $RP/logs/train_eval.log
log "train m1 start"
$PY -m rp.train --data $RP/extracted --out $RP/runs/m1 \
  --m-hyp 1 --epochs 200 --batch 64 --workers 0 --margin-lambda 3 --margin 0.0625 \
  2>&1 | tail -8 | tee -a $RP/logs/train_eval.log
log "train m3norule start"
$PY -m rp.train --data $RP/extracted --out $RP/runs/m3norule \
  --m-hyp 3 --zero-rule --epochs 200 --batch 64 --workers 0 --margin-lambda 3 --margin 0.0625 \
  2>&1 | tail -8 | tee -a $RP/logs/train_eval.log

log "eval start"
$PY -m rp.evaluate --data $RP/extracted \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --m3 $RP/runs/m3/best.pt --m1 $RP/runs/m1/best.pt \
  --m3norule $RP/runs/m3norule/best.pt \
  --out $RP/eval --rollout 2>&1 | tail -70 | tee -a $RP/logs/train_eval.log

log "replay scan start"
$PY $RP/tools/replay_sample.py --data $RP/extracted \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --weights $RP/runs/m3/best_fused.npz \
  --out $RP/eval/replay_h1fail_h2ok.json \
  2>&1 | tail -30 | tee -a $RP/logs/train_eval.log
log "train+eval done"
