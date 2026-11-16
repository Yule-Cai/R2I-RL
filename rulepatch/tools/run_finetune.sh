#!/bin/bash
# Fine-tune m3 on small-rect + orig mix (multi-scale detours for Gazebo).
# Usage: bash run_finetune.sh
set -u
RP=/home/yule/rulepatch
PY=/home/yule/miniforge3/envs/causal-repair/bin/python
export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:${PYTHONPATH:-}
export OMP_NUM_THREADS=4
log() { echo "[ft] $(date '+%H:%M:%S') $*" | tee -a $RP/logs/finetune.log; }

log "extract small start"
$PY $RP/tools/run_extract.py --data $RP/small \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --out $RP/small_extracted 2>&1 | tail -12 | tee -a $RP/logs/finetune.log

log "combine splits"
rm -rf $RP/combined && mkdir -p $RP/combined
cd $RP/extracted && for f in train_*.npz val_*.npz test_*.npz; do
  ln -sf ../extracted/$f $RP/combined/$f
done
cd $RP/small_extracted && n=0
for f in train_*.npz; do
  # 3x oversample: small detours must not drown in orig volume
  for rep in s t u; do
    ln -sf ../small_extracted/$f $RP/combined/train_${rep}$(printf %04d $n).npz
  done
  n=$((n + 1))
done
n=0
for f in val_*.npz; do
  ln -sf ../small_extracted/$f $RP/combined/val_s$(printf %04d $n).npz
  n=$((n + 1))
done
ls $RP/combined/ | tee -a $RP/logs/finetune.log

log "finetune m3 start"
$PY -m rp.train --data $RP/combined --out $RP/runs/m3small \
  --m-hyp 3 --epochs 25 --batch 64 --workers 0 --lr 1e-4 \
  --margin-lambda 3 --margin 0.0625 --init $RP/runs/m3/best.pt \
  --seed 1 2>&1 | tail -8 | tee -a $RP/logs/finetune.log

log "eval warehouse start"
$PY -m rp.evaluate --data $RP/extracted \
  --pgm /opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm \
  --m3 $RP/runs/m3small/best.pt --out $RP/eval_small --rollout \
  2>&1 | tail -40 | tee -a $RP/logs/finetune.log
log "finetune done"
