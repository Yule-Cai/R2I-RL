# Retained component contract tests

These are unchanged existing authorization and RuleGuard tests. The two modules are byte-identical to the pinned navigation runtime. They run without ROS/Gazebo, with Python, pytest, and NumPy installed:

```sh
cd evidence/component_contracts
python3 -m pytest tests -q
```

The manuscript reports these component checks separately from real ROS message probes. `python3 verify_binding_latch.py` rechecks three captured-message mutations and the existing diagnostic latch records from the bundled raw archives using the frozen production verifier and rejection methods with recording infrastructure.
