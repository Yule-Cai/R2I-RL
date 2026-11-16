<h1 align="center">RuleSync</h1>

<p align="center"><strong>Durable authorization for modular robot navigation</strong></p>
<p align="center">Connect the committed rule state to the path released for execution.</p>

<p align="center">
  <a href="#quick-start">Quick start</a> ·
  <a href="#architecture">Architecture</a> ·
  <a href="#ros-2-integration">ROS 2 integration</a> ·
  <a href="#data-and-reproduction">Data &amp; reproduction</a> ·
  <a href="docs/README_ZH.md">中文导读</a>
</p>

<p align="center"><a href="docs/assets/rulesync-overview.svg"><img src="docs/assets/rulesync-overview.svg" alt="RuleSync architecture: durable snapshot selection, continuous path admission, content binding, and independent execution monitoring with renewed authorization after HOLD." width="1100"></a></p>

**A path can remain geometrically clear after the rule decision authorizing it has changed.** RuleSync carries the identity of a complete, durable spatial-rule snapshot through path admission and execution release. An independent receiver checks the authorization against usable observed state and actual pose. Invalidation latches HOLD; motion resumes after fresh admission supplies a valid new authorization.

RuleSync integrates with ROS 2/Nav2 while retaining existing planners and controllers. Source code, experimental records and reproduction tools are available directly on the default **main** branch.

| Navigation records | Furnished layouts | Paired admission measurements | Pinned data sources |
|:---:|:---:|:---:|:---:|
| **2,970 attempts** | **23 layouts** | **720 pairs** | **93 files** |

## Quick start

The offline checks run on Linux or macOS with **Python 3.10+**. They do not require ROS, Gazebo or robot hardware.

```bash
git clone https://github.com/Yule-Cai/R2I-RL.git
cd R2I-RL

python3 -m venv .venv-rulesync
source .venv-rulesync/bin/activate
python -m pip install -r requirements-rulesync.txt

bash scripts/check_rulesync_offline.sh
python scripts/verify_jsa_release.py
bash scripts/check_jsa_harnesses.sh
```

| Command | What it checks |
|:---|:---|
| `check_rulesync_offline.sh` | Geometry, authorization content, monitor state, transaction recovery and helper contracts |
| `verify_jsa_release.py` | Repository/data hashes, fixed campaign inputs, original receiver records and recomputed measurement endpoints |
| `check_jsa_harnesses.sh` | Frozen campaign and observer-ledger contracts, with their module paths configured explicitly |

The verification scripts print their results to the terminal. Data-verification receipts are written under `artifacts/jsa_20261116/`; detailed component receipts use its `qa/` directory. ROS-dependent tests are skipped when the ROS environment is unavailable.

## Architecture

RuleSync connects three questions: which rule snapshot was selected, which path was admitted under it, and whether that authorization can be released with the receiver's current observations.

| Stage | Responsibility | Implementation entry point |
|:---|:---|:---|
| **Durable rule selection** | Retain a complete rule snapshot using WAL-backed COMMIT and reconstruct the selected target after restart | [Manager](rulesync/src/rulesync_manager/rulesync_manager/manager_node.py) · [Coordinator](rulesync/src/rulesync_manager/rulesync_manager/vote_coordinator.py) |
| **Participant reconciliation** | Establish agreement on the selected snapshot epoch and content hash | [Nav2 bridge](rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py) |
| **Continuous admission** | Check every candidate path segment against the inflated forbidden regions | [Planar gate](rulesync3d/rulesync3d/gate2d.py) · [ROS gate](topopatch_ros/topopatch_ros/gate_node.py) |
| **Content-bound authorization** | Carry the path, rule identity and path digest together | [Authorization helpers](rulesync3d/rulesync3d/authorization.py) · [Typed message](rulesync/src/rulesync_interfaces/msg/ExecutionAuthorization.msg) |
| **Verified release and renewal** | Check content, identity, observation freshness, frame, actual pose, clearance and path-start connection; require new authorization after invalidation | [RuleGuard adapter](rulesync/src/rulesync_guard/rulesync_guard/guard_node.py) · [Monitor core](rulesync3d/rulesync3d/ruleguard.py) |

The snapshot identity is `(epoch, snapshot_hash)`. A delayed authorization can therefore be refused even when its path geometry remains clear. After invalidation, receiving another usable snapshot alone does not reopen the output: a fresh authorization must pass the receiver checks.

The receiver acts on its fresh, usable observed snapshot. Durable COMMIT, observation, HOLD, renewed release and task arrival are distinct recorded events.

## ROS 2 integration

### Environment

| Layer | Requirements |
|:---|:---|
| Offline verification | Python 3.10+, NumPy, pytest and Shapely; see [requirements-rulesync.txt](requirements-rulesync.txt) |
| ROS integration | Linux with ROS 2, colcon and rosdep; the recorded evaluation used **ROS 2 Lyrical** |
| Navigation and simulation | Nav2, Gazebo and the matching ROS/Gazebo integration packages |
| Furnished-scene campaigns | Acquired HSSD assets, the fixed case inputs and the recorded runner configuration |

Install ROS/Nav2 using the [official Nav2 installation guide](https://docs.nav2.org/lyrical/getting_started/build_and_install/local_installation/). The Python requirements file installs the offline dependencies; ROS and simulator packages are managed through the ROS/system environment.

### Build in a separate workspace

Use a fresh ROS terminal with the system Python environment. From the cloned repository root, with ROS and rosdep already configured, create a new workspace:

```bash
source /opt/ros/lyrical/setup.bash
rulesync_repo="$PWD"
rulesync_ws="$(dirname "$rulesync_repo")/rulesync_ws"
mkdir -p "$rulesync_ws/src"

for pkg in rulesync_interfaces rulesync_manager rulesync_nav2_bridge rulesync_guard rulesync_demo; do
  ln -s "$rulesync_repo/rulesync/src/$pkg" "$rulesync_ws/src/$pkg"
done
for pkg in rulesync3d topopatch_ros rulepatch_ros; do
  ln -s "$rulesync_repo/$pkg" "$rulesync_ws/src/$pkg"
done

cd "$rulesync_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export PYTHONPATH="$rulesync_repo${PYTHONPATH:+:$PYTHONPATH}"
```

Use a new workspace so the symlink commands do not overwrite another checkout. The repository root is included in `PYTHONPATH` for the retained Python compatibility modules.

### Connect the runtime

1. Bring up the navigation stack, map, localization and pose stream for the selected world.
2. Start the durable manager and Nav2 bridge:

   ```bash
   ros2 launch rulesync_demo rulesync_sim.launch.py
   ```

3. Start the independent execution monitor in another sourced terminal:

   ```bash
   ros2 launch rulesync_demo rulesync_guard.launch.py \
     pose_topic:=/ruleguard/pose \
     executable_topic:=/rulepatch/executable_path
   ```

4. Route the controller's path input to RuleGuard's executable output. Supply localized `PoseStamped` messages in the committed rule frame.

`rulesync_sim.launch.py` starts the manager and bridge. The world, Nav2 stack, gate and controller are supplied by the selected integration or experiment runner. The [ROS package guide](rulesync/README.md) describes the interfaces and launch responsibilities.

### Interfaces and configuration

| Interface | Role |
|:---|:---|
| `/rulesync/propose_rule`, `/rulesync/commit_rule`, `/rulesync/remove_rule` | Planar rule-update lifecycle |
| `/rulesync/propose_volume`, `/rulesync/commit_volume`, `/rulesync/remove_volume` | Volumetric rule-update lifecycle |
| `/rulesync/list_rules`, `/rulesync/list_volumes` | Rule-state observations |
| `/rulesync/execution_authorization` | Complete admitted path and bound identity/content fields |
| `/rulesync/ruleguard_state` | CLEAR/WARNING/HOLD state and identity evidence |
| `/rulepatch/executable_path` | Default receiver-verified controller-facing path output |

The following are defaults in the current source. Frozen experiments retain their own recorded parameters.

| Parameter | Default | Purpose |
|:---|:---:|:---|
| `rule_frame` | `map` | Common rule and pose frame |
| `rho` | `0.20 m` | Geometric inflation/clearance radius |
| `snapshot_poll_s` | `0.20 s` | Snapshot-query period |
| `snapshot_stale_s` | `1.00 s` | Maximum usable observation age |
| `snapshot_request_timeout_s` | `1.00 s` | Snapshot-query timeout |
| `pose_stale_s` | `0.50 s` | Maximum pose age |
| `path_start_tolerance` | `0.25 m` | Upper bound for the actual-pose-to-path-start connection |
| `use_polygon_index` | `false` | Optional snapshot-keyed geometry index in the gate |
| `publish_legacy_path` | `false` | Compatibility output used by specified frozen harnesses |

See the [receiver parameters](rulesync/src/rulesync_guard/rulesync_guard/guard_node.py) and [gate parameters](topopatch_ros/topopatch_ros/gate_node.py) for the exact checks. The independent admission predicate covers hard KEEP_OUT geometry. Speed-limit fields also participate in snapshot identity and Nav2 propagation; a geometric path does not specify a time-parameterized speed profile.

## Data and reproduction

The [artifact index](artifacts/jsa_20261116/README.md) describes the retained files. The [reproduction guide](docs/RULESYNC_REPRODUCIBILITY_20261116.md) maps each study to its inputs and checks.

| Collection | Included records | Entry point |
|:---|:---|:---|
| Protocol recovery | 36 planar/volumetric checks | [2D matrix](artifacts/jsa_20261116/source_data/protocol_matrix_2d.csv) · [3D matrix](artifacts/jsa_20261116/source_data/protocol_matrix_3d.csv) |
| Receiver probes | 12 typed-message attempts and the original 500-file archive | [Archive](artifacts/jsa_20261116/receiver12_raw.tar.gz) · [Per-trial assessment](artifacts/jsa_20261116/source_data/authorization12_review.json) |
| Attribution diagnostics | Three separately counted source-pinned archives | [Raw archives](artifacts/jsa_20261116/source_data/diagnostic_raw/) |
| Navigation | All 2,970 trial rows across 23 furnished layouts | [Trial-level CSV](artifacts/jsa_20261116/source_data/formal2970.csv) |
| Admission cost | 720 paired measurements, warmups and condition summaries | [Measurement records](artifacts/jsa_20261116/source_data/ros_admission_performance/) |
| Recovery timing | Per-trial renewal endpoints and missing-data accounting | [Timing rows](artifacts/jsa_20261116/source_data/formal_recovery_timing_rows.json) |
| Moving update | One matched update during motion, with recorded command/observation events | [Trace records](artifacts/jsa_20261116/source_data/moving_update/) |
| Volumetric assessment | 140 original outcomes and active-snapshot assessments | [Assessment records](artifacts/jsa_20261116/source_data/volumetric140_review.json) |
| Fixed campaign inputs | 197 maps, cases, world references and helper sources | [Frozen inputs](artifacts/jsa_20261116/frozen_inputs/) |

Original task failures and unassessed intervals remain explicit. The included numerical data and selected raw records can be checked locally. The full bulk navigation/volumetric motion logs and upstream HSSD meshes are retained separately; acquire source scenes through the dataset's distribution terms.

### Regenerate numerical visualizations

From the repository root, with the offline environment active:

```bash
python -m pip install matplotlib pillow
python scripts/reproduce_jsa_figures.py --out output/jsa-regenerated-figures
```

The output directory must be new. The wrapper runs in a temporary workspace and preserves the pinned inputs.

### Experiment runners

All active runners are under [experiments/rulesync_experiments/](experiments/rulesync_experiments/).

- [Formal campaign](experiments/rulesync_experiments/formal_dualmap_20261108_v2/): frozen contracts, queues and navigation runners.
- [Message diagnostics](experiments/rulesync_experiments/authorization_diagnostic_20261115/): observer ledgers, probe logic and diagnostic runners.
- [Snapshot-aligned volumetric study](experiments/rulesync_experiments/quad3d_snapshot_dev_20261114_v3/): volumetric geometry and audit sources.

Archived launchers record their acquisition-host paths, images and mounts. Configure an isolated reproduction root and acquired scene assets before running a ROS/Gazebo campaign; preserve the fixed inputs and seed matrix.

## Repository map

```text
R2I-RL/
├── rulesync/                  ROS 2 interfaces, manager, bridge and RuleGuard
├── rulesync3d/                Geometry, content binding and monitor core
├── topopatch_ros/             Path admission and navigation integration
├── rulepatch_ros/             Controller/navigation compatibility components
├── topopatch/ · rulepatch/    Retained Python dependencies
├── experiments/
│   └── rulesync_experiments/  Frozen campaign and diagnostic harnesses
├── scripts/                   Tests, verification and analysis tools
├── artifacts/
│   └── jsa_20261116/          Source data, fixed inputs and raw records
└── docs/                      Integration and reproduction guides
```

## Common questions

<details>
<summary><strong>Do I need ROS to check the data?</strong></summary>

No. Install the offline requirements and run the quick-start checks. ROS/Gazebo are needed for the live integration and navigation campaigns.

</details>

<details>
<summary><strong>Why does execution remain in HOLD after a fresh snapshot arrives?</strong></summary>

Invalidation latches the receiver output. A usable observation supplies rule-state evidence; a new path authorization supplies permission to resume. Re-admit the candidate and send the resulting complete authorization through the normal receiver interface.

</details>

<details>
<summary><strong>Where are verification results and generated plots written?</strong></summary>

`scripts/verify_jsa_release.py` writes `artifacts/jsa_20261116/verification_latest.json` and component receipts under `qa/`. The figure wrapper writes to the new directory passed through `--out`.

</details>

## Citation and contact

Use [CITATION.md](CITATION.md) to reference the software artifact. The default branch provides the current layout; the retained `rulesync-jsa-20261116` tag remains an entry point for earlier versioned references. Dependency and dataset terms apply to their respective materials.

**Maintainer:** Yule Cai · [cai_yule@soc.uum.edu.my](mailto:cai_yule@soc.uum.edu.my)
