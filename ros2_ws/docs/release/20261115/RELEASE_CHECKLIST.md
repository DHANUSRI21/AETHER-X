# AETHER-X — Release Checklist

## Already verified in the 15 November 2026 session

- [x] Full workspace `colcon build`: 7 packages finished.
- [x] Full workspace `colcon test`: 7 packages finished.
- [x] Aggregate test result: 86 tests, 0 errors, 0 failures, 8 skipped.
- [x] Single-instance base bringup verified.
- [x] `joint_state_broadcaster` and `diff_drive_controller` active.
- [x] Four wheel joints present in `/joint_states`.
- [x] Core status heartbeat published; monitor reached OPERATIONAL.
- [x] IMU/GNSS/LiDAR measured at approximately 100/10/10 Hz.
- [x] SLAM Toolbox configured and active; `/map` published.
- [x] Nav2 planner/controller/BT/behavior/smoother lifecycle nodes active.
- [x] Path-only planning action succeeded.
- [x] NavigateToPose succeeded with zero recovery behaviours.
- [x] Controller output verified zero after navigation.
- [x] Runtime snapshot command saved a 2.9 KB `runtime_snapshot.txt` on the workstation.

## Still open before calling this a formal release

- [ ] Identify the eight skipped tests and document skip reasons.
- [ ] Investigate/resolve the `Enforcing command limits is disabled` warning.
- [ ] Capture a fresh screenshot of the 15 November running system.
- [ ] Attach the live `runtime_snapshot.txt` and complete launch logs to the evidence folder.
- [ ] Preserve `colcon test-result --verbose` output and test XML reports.
- [ ] Run `git status` and ensure no build/install/log artifacts are accidentally committed.
- [ ] Decide whether to execute the dynamic-obstacle rerouting test or list it as future work.
- [ ] Create release tag/commit only after the open release gates are reviewed.
