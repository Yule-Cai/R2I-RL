# Reproducing the RuleSync research artifact

## Offline verification

Install `requirements-rulesync.txt`, then run `bash scripts/check_rulesync_offline.sh`. To verify the research records and added artifact hashes without ROS/Gazebo, run `python scripts/verify_jsa_release.py`. These commands do not start experiments. The frozen campaign/observer harness tests run with `bash scripts/check_jsa_harnesses.sh`; that wrapper sets their archived module search paths explicitly.

Component checks are available under `artifacts/jsa_20261116/evidence/component_contracts`. They are the original 26 authorization/monitor tests using byte-identical frozen modules.

## Numerical visualizations

Numerical figures can be regenerated from the retained data into a new output directory:

```sh
python -m pip install matplotlib pillow
python scripts/reproduce_jsa_figures.py --out output/jsa-regenerated-figures
```

The wrapper executes the plotting code on pinned CSV/JSON and native recaptures in a temporary workspace. It preserves the retained data. Summary reference cells are stored in CSV; source-data verification does not require a manuscript, LaTeX or a PDF.

## Evidence map

| Study | Included records | Independent checks |
|---|---|---|
| Protocol recovery (36) | Planar/volumetric matrix CSVs and pinned runtime | Manager/bridge identity agreement and exact row accounting |
| Typed receiver probes (12) | Original 500-file archive, per-trial review and frozen message helpers | Archive member hashes, explicit old-identity refusals, preserved 8/4 task outcomes |
| Attribution diagnostics (3) | Complete original archives, source pins and ledgers | Refusal, no stale-handler nonempty publication, fresh authorization/release; usable-observation counts 11/11/8 |
| Moving update (one matched pair) | Manager/guard/WAL events, command publication timeline, response decoding and audit | COMMIT/observation/HOLD timings and recorded publication counts |
| Admission cost (720 pairs) | Raw JSONL, 120 warmup pairs, 24 condition summaries and pinned sources | Verdict/content identity agreement and response/compute quantiles |
| Navigation (2,970 attempts) | All-row CSV, outcome/coverage records, renewal endpoints; exact input manifest and 197 frozen input files | Cohort Cartesian coverage, original task labels, matched-arm counts and retained missing endpoints |
| Volumetric study (140 attempts) | Per-row original outcomes and active-snapshot reanalysis, occupancy comparisons, pinned DEV sources | 110/30 original labels and 130/10 assessment accounting |

The Git artifact does not contain the full large navigation log corpus or all original volumetric motion traces. Their retained archive identities/audit summaries identify the evidence, but aggregate verification here is not a new raw-geometry reanalysis of those omitted traces. Those bulk files need a separate accessible data deposit before a complete external replication claim. No source meshes are bundled: obtain HSSD using its official dataset distribution and applicable upstream terms, then resolve the world/model references against the acquired assets. Derived maps and case definitions remain hash-pinned here.

## ROS/Gazebo reproduction

See `rulesync/README.md` for the ROS workspace build and message/service interfaces. The campaign used the recorded ROS 2 Lyrical/Nav2/Gazebo environment; image identifiers and resource settings are preserved in manifests. Prepared inputs are under `artifacts/jsa_20261116/frozen_inputs/`, retaining their original relative path layout. The manifest is `formal_dualmap_20261108_v2/manifest.json` (SHA-256 bb749f5d886ef5a0009966ce4d180b3c58e2c1d503213a918cedd94534dfef20). Launchers and formal runners are under `experiments/rulesync_experiments/formal_dualmap_20261108_v2/`.

Archived launchers retain the recorded host paths, image identifiers and mounts for provenance. Configure a separate reproduction root and compatible image/assets on a new machine; do not execute the original launch commands against an existing experiment output directory. Inputs, scientific runtime methods and seed matrix should remain pinned. No full ROS/Gazebo campaign was rerun during this repository preparation.

## Version, access and reuse

The main branch provides source code, experimental records and verification tools directly. The `rulesync-jsa-20261116` tag remains available for versioned references. Code and included records are publicly accessible. Upstream dependency and dataset terms remain applicable; no new project-wide license is asserted by this release.
