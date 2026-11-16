# RulePatchNet (Milestone 2) — rule-conditioned local path repair for Nav2

Learns to patch only the affected local segment when a `KEEP_OUT` RuleSpec
is committed, instead of full replanning or full `cmd_vel` prediction.

```
 Nav2 global path + committed KEEP_OUT -> A/B anchors -> 128x128 BEV
 (occupancy / original-path / rule / goal) + 32x2 local points
   -> RulePatchNet (<=3M params): 3 x (6 spline control points + confidence)
   -> deterministic verifier -> accepted patch | fallback Nav2 full replan
```

## Layout

- `rp/` — pure-python core (`numpy`; `torch` only for `model`/`train`):
  `geom` anchors+splines, `bev` rendering, `verifier` deterministic checks,
  `model` RulePatchNet, `infer_numpy` torch-free forward (ROS node),
  `dataset` shards/splits, `extract` A/B patch extraction,
  `train` supervised WTA training, `evaluate` baselines + metrics.
- `tools/gen_dataset.py` — real-Nav2 teacher generator (onexplayer,
  `ROS_DOMAIN_ID=7` headless planner stack).
- `tests/` — `pytest` suite (no ROS).

## Quick check (no ROS)

```bash
PYTHONPATH=rulepatch python3 -m pytest rulepatch/tests -q
PYTHONPATH=rulepatch python3 -m rp.train --data <extracted> --out runs/m3
PYTHONPATH=rulepatch python3 -m rp.evaluate --data <extracted> \
  --pgm warehouse.pgm --m3 runs/m3/best.pt --out eval_m3 --rollout
```

## ROS

`rulepatch_ros/` (top level): `rulepatch_inference`, `rulepatch_auto_repair`
(auto replan + post-commit validation + rollback), `rulepatch_pp_driver`.
`rp/` is provided via `PYTHONPATH=<repo>/rulepatch`.
