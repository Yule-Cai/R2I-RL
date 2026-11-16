"""RuleSync B1 quadrotor 3D DEV package (IoT-J migration, 2026-11).

Pure-Python 3D rule-authority and geometry helpers used by
``scene_runner_quad3d_v1.py``.  The authoritative 3D geometry predicates are
imported from the installed ``rulesync3d`` package (the same module build that
is sha256-pinned by the launcher preflight); this package adds the durable
volume-commit envelope, the case/occupancy adapters and the audit helpers.

No ROS imports at package import time so the offline self-test
(``selftest_quad3d_v1.py``) runs without a ROS environment.
"""
