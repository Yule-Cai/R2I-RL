# Final moving-update evaluation

**36 attempts · 18 matched pairs · 36 renewed releases · 35 task arrivals**

This collection follows the authorization lifecycle while a robot is executing an existing path. Three routes and three seeds are crossed with clean/2.5 s response-omission conditions and Full/geometry-only monitoring. Route 1 changes the durable epoch while retaining admissible path geometry; routes 2 and 3 introduce replacement regions intersecting the old path after inflation.

| Endpoint | Recorded result |
|:---|:---|
| Completed and independently assessed attempts | 36/36 |
| Matched settings | 18/18 |
| Renewed execution release | 36/36 |
| Arrival within the fixed 0.20 m independent tolerance | 35/36 |
| Clean Full: identity-triggered HOLD | 9/9; median 85.8 ms after COMMIT |
| Clean geometry-only: old execution retained under the new observed authority | 9/9 |
| Omission Full: observation-expiry HOLD | 9/9; median 989.9 ms after COMMIT |
| Final command publication/value/simulator-bridge-take associations | 58,211 |
| Sampled body entries into the active rule region during the measured window | 0/36 |

The remaining task reports controller completion with an independent goal error of 0.210827 m, outside the fixed 0.20 m criterion. Its original FAIL label remains in [trial_results.csv](trial_results.csv) and the raw record. Arrival, renewed authorization, sampled motion and command timing are separate endpoints.

## Verify without ROS

From the repository root, use Python 3.9 or newer:

```bash
python3 scripts/verify_final_moving_update.py
```

This standard-library-only command verifies file hashes and frozen inputs, reconstructs native command publication/take/value associations, compares independently decoded service-response fields, and recomputes timing and sampled-body endpoints. It does not launch ROS, simulation or a robot experiment. Independent acquisition-host and macOS receipts are retained in [verification/](verification/).

## Browse the records

| Path | Contents |
|:---|:---|
| [summary.json](summary.json) | Compact endpoint summary |
| [trial_results.csv](trial_results.csv) | All 36 attempts with original task labels |
| [final36/manifest.json](final36/manifest.json) | Fixed matrix, seed order, source hashes and resource settings |
| [final36/aggregate.json](final36/aggregate.json) | Per-condition timing distributions and per-attempt assessments |
| [final36/runs/](final36/runs/) | Original outcomes, WAL, monitor/adapter events, sampled poses, service serializations and native-record subsets |
| [final36/source/](final36/source/) | Frozen runtime instrumentation and qualified response-forwarding harness |
| [final36/analysis/](final36/analysis/) | Independent decoder, native/window/geometry auditors and regression checks |
| [final36/artifact_files.json](final36/artifact_files.json) | Byte counts and SHA-256 hashes for the exported evidence |

The acquisition configuration is one worker, the recorded ROS image, 3 CPU quota and a 3 GiB container limit. The moving exposure lasts five seconds after durable COMMIT; both monitor configurations then use the same stop/replan/readmit policy. Frozen launchers retain their acquisition-host mount paths and image identity. Re-execution requires the recorded environment and acquired scene assets; offline evidence verification uses the records supplied here.

HOLD-to-stable-zero publication has a median of 46.1 ms across 18 Full runs, with an observed range of 35.4–91.2 ms. Three Full runs and one geometry-only run contain a nonzero publication after HOLD before the stream becomes zero. These are software command endpoints; sampled body poses provide a separate motion measurement.

The native subsets retain unchanged records for the relevant command/authorization topics and endpoint initialization. Full original CTF/native traces remain separately stored and are identified by their hashes. This collection does not contain every middleware event or redistribute upstream scene assets. The final cohort remains separate from earlier development and harness-qualification records.
