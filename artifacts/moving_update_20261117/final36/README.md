# RuleSync moving-update evidence

This archive supports independent recomputation of the moving-update experiment.
`manifest.json` fixes the 36 planned attempts and 18 pairs. Every exported run
includes raw WAL, manager/guard/adapter events, sampled poses, snapshot responses,
the frozen runtime extracts, exact final-command correlations and original outcomes.

Run the evidence verification with Python 3.9 or newer:

```sh
python3 analysis/verify_artifact.py
```

No ROS installation, simulation or external Python dependency is needed for this
verification. The retained independent decoder can additionally deserialize the
captured application CDR using the pinned ROS image; decoded fields and canonical
hashes are supplied here. These are application serializations, not packet capture.

`native_relevant.jsonl` contains unchanged original records for command and
authorization topics plus all endpoint initialization records. The subset verifier
reconstructs publication/take/value associations and compares the complete command
timeline with the independently audited full trace. Every subset links the full
native trace SHA-256 and its unmodified trace metadata. Full CTF and native traces
are retained separately; this archive does not claim to contain every middleware event.

The time window ends five seconds after COMMIT. The common stop at its end is
separate from measured monitor intervention. Callback entry, HOLD, command
publication and simulator middleware receipt are software endpoints. Sampled body
nonentry and task arrival are distinct outcomes; failed tasks remain failed.

Requests before initial release or after the measured window without a response
are indexed separately from the active experiment. Every active-period request and every captured response
used by the measured impairment is verified. Historical receiver-probe outcomes
and their separate diagnostics are linked by `analysis/observer_trace_index.csv`.

The original simulator assets are identified by `preflight.json` and the retained
scene source manifests. Obtain them from their upstream distributions under their
original licenses. `campaign.py` records the workstation-specific execution setup;
the evidence verification above does not launch that campaign.
