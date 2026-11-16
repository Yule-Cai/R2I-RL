# RulePatchNet Milestone 2 — rule-conditioned local path repair for ROS2/Nav2.
#
# Pure-python core (numpy only; torch needed only for rp.model / rp.train).
# ROS-dependent drivers live in rulepatch/tools/ (run on onexplayer) and the
# rulepatch_ros/ package. No SemanticNav2D, no SFD/VARI/SMI/TRP/HARS code.

TRAIN_SPLIT = "train"
VAL_SPLIT = "val"
TEST_SPLIT = "test"

# Held-out test region: warehouse map x greater than this (east strip).
TEST_REGION_X_MIN = 9.0

# BEV window geometry.
BEV_PX = 128
BEV_SIZE_M = 12.8
BEV_RES = BEV_SIZE_M / BEV_PX  # 0.1 m/px

# Anchor extraction.
DIV_THRESH = 0.35      # divergence distance (m)
REJOIN_TOL = 0.175     # endpoint rejoin tolerance (m)
MIN_PATCH_STRAIGHT = 1.0   # min straight-line A->B distance (m)
MAX_DETOUR_RATIO = 2.5     # max repl_len / orig_len for a sane teacher patch

# Spline / model shapes.
N_CTRL = 6
N_HYP = 3
N_LOCAL_PTS = 32
