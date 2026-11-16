# RuleSync ROS 2 packages

This directory contains the ROS 2 implementation of RuleSync's durable rule
deployment path:

```text
structured RuleSpec / RuleVolume candidate
  -> deterministic validation
  -> PREPARE with (txn_id, epoch, target_hash)
  -> all participants READY
  -> durable WAL COMMIT (flush + fsync; linearization point)
  -> certificate-bound application of the exact prepared snapshot
  -> explicit convergence on (epoch, snapshot_hash)
```

The transaction manager is the authority. Participant state is evidence only;
recovery never promotes an uncommitted bridge state.

## Packages

- `rulesync_interfaces`: messages and services for 2-D rules and 3-D volumes.
- `rulesync_manager`: validation, durable WAL/checkpoint, 2PC, recovery, and
  reconciliation.
- `rulesync_nav2_bridge`: Nav2 keepout/speed-filter participant and applied-state
  query.
- `rulesync_guard`: independent manager-snapshot observer and execution-time
  monitor; it is the controller-facing path release point.
- `rulesync_demo`: simulation and manually initiated demo clients.

RuleGuard's deterministic core and authorization digest are in
`../rulesync3d/rulesync3d/`. The ROS adapter integrates path authorization with independently observed rule state and actual pose. Current experiment endpoints, reproduction commands, and data coverage are described in [the reproduction guide](../docs/RULESYNC_REPRODUCIBILITY_20261116.md).

## Build

The measured environment used ROS 2 Lyrical. Create a workspace and link or copy
the packages under `rulesync/src/`, plus the repository's `rulesync3d/` and
`topopatch_ros/` directories, into its `src/` directory, then run:

```bash
source /opt/ros/lyrical/setup.bash
cd /path/to/rulesync_ws
colcon build --symlink-install
source install/setup.bash
```

Nav2/Gazebo dependencies are system packages and are not installed by the
repository's Python requirements file.

## Launch

```bash
ros2 launch rulesync_demo rulesync_sim.launch.py
```

The real-robot launch file is configuration-only. No motion starts automatically:

```bash
ros2 launch rulesync_demo rulesync_real.launch.py
```

The independent execution monitor is launched separately so adding it cannot
silently alter an existing robot controller:

```bash
ros2 launch rulesync_demo rulesync_guard.launch.py \
  pose_topic:=/ruleguard/pose \
  executable_topic:=/rulepatch/executable_path
```

`pose_topic` must be a `geometry_msgs/PoseStamped` stream in the committed rule
frame (default `map`). A production controller enters the new chain only after
its path subscription is explicitly remapped to `executable_topic`. The gate's
legacy `/rulepatch/gated_path` publication is disabled by default; frozen-harness
reproduction must explicitly set `publish_legacy_path:=true`.

## Offline tests

From the repository root:

```bash
python -m pip install -r requirements-rulesync.txt
bash scripts/check_rulesync_offline.sh
```

## Runtime services

- `/rulesync/propose_rule`, `/rulesync/commit_rule`, `/rulesync/remove_rule`
- `/rulesync/list_rules`
- `/rulesync/propose_volume`, `/rulesync/commit_volume`, `/rulesync/remove_volume`
- `/rulesync/list_volumes`
- `/rulesync/execution_authorization` (atomic admitted path + epoch/hash/digest)
- `/rulesync/ruleguard_state` (CLEAR/WARNING/HOLD and both identities)
- internal bridge vote/state services for reconciliation

## Claim boundary

KEEP_OUT rules are covered by the geometric admission predicate and RuleGuard.
SPEED_LIMIT values participate in validation, canonical snapshot identity, and
Nav2 filter propagation, but the current independent gate and RuleGuard do not
verify a time-parameterized speed profile. The candidate is geometric; speed/time-profile enforcement is a separate deployment concern.
