# TopoRulePatch Milestone 3 — topology-aware verified runtime-rule path repair.
#
# New package. M2 (`rulepatch/`, `rulepatch_ros/`, `rulesync/`) is FROZEN:
# nothing under those trees may change behavior. This package only READS
# M2 modules (geom/bev/verifier as pure-python libs) and M2 checkpoints
# (as frozen baselines). No SemanticNav2D, no SFD/VARI/SMI/TRP/HARS code.
#
# Conventions shared across topopatch modules are documented in
# topopatch/topo/labels.py (topology representation) and
# topopatch/config/freeze.json (frozen protocol hashes, written at freeze).

TOPO_SIDE_NEG = 0
TOPO_SIDE_POS = 1
TOPO_AMBIGUOUS = 2
TOPO_NAMES = ("SIDE_NEG", "SIDE_POS", "AMBIGUOUS")
N_TOPO_CLASSES = 3
