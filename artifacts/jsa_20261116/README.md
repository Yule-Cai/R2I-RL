# RuleSync experimental data and reproduction artifact

- `source_data/`: 93 hash-pinned scientific data files, including the 2,970 navigation rows, 720 paired admission measurements, recovery endpoints, protocol and volumetric records, and three raw diagnostic archives.
- `source_data_manifest.json`: byte counts and SHA-256 hashes for those 93 files.
- `receiver12_raw.tar.gz`: original twelve typed-message attempts; 500 inventoried files with original outcomes retained.
- `frozen_inputs/`: 197 fixed input files, including route cases, layouts, maps, world references and helper sources.
- `evidence/`: frozen implementation extracts, component tests and expected binding/latch checks.
- `recovery_timing_reference.csv`, `admission_cost_reference.csv`: retained numerical reference cells, independently checked against original measurement rows.
- `release_manifest.json`: inventory of the current code/data snapshot.

From the repository root run `python scripts/verify_jsa_release.py`. It verifies file hashes and recomputes the recorded endpoints without requiring a manuscript or ROS/Gazebo. Full bulk motion logs and upstream meshes require separate storage; see the reproduction guide for exact data coverage.
