# AETHER-X — Evidence Index

## Evidence recorded during 15 November 2026 runtime smoke test

| Evidence | Location / result | Status |
|---|---|---|
| Full build | Terminal output: `Summary: 7 packages finished [1min 19s]` | Captured in conversation |
| Full test result | `86 tests, 0 errors, 0 failures, 8 skipped` | Captured in conversation |
| Base nodes/controllers | Single instance of each base node; both controllers active | Captured in conversation |
| Wheel joints | `/joint_states` contained four wheel joints | Captured in conversation |
| Sensor rates | IMU ~100 Hz, GNSS ~10 Hz, LiDAR ~10 Hz | Captured in conversation |
| Localization/SLAM | EKF/GPS/map topics; SLAM lifecycle active; 215 x 194 map at 0.05 m | Captured in conversation |
| Nav2 | Five lifecycle nodes active; actions and costmaps present | Captured in conversation |
| Path planning | ComputePathToPose succeeded; ~3.03 ms planning time | Captured in conversation |
| Goal execution | NavigateToPose succeeded; 0 recoveries; ~0.197 m straight-line final position error | Captured in conversation |
| Stop verification | Odom twist approximately zero; cmd_vel_out exactly zero | Captured in conversation |
| Runtime snapshot | `/mnt/d/AETHER-X/evidence/runtime_smoke_20261115/runtime_snapshot.txt` | Saved on workstation; not embedded in this kit |

## Included supplementary visualization

- `screenshots/nav2_lidar_costmap_rviz_20261114.png` — RViz screenshot from 14 November 2026 showing the robot model, LiDAR scan display and navigation costmap visualization. It is supplementary and predates the 15 November runtime smoke test.

## Add the live runtime snapshot to this folder

From WSL, if this kit is extracted to `/mnt/d/AETHER-X/Release-Evidence-Kit`, run:

```bash
mkdir -p /mnt/d/AETHER-X/Release-Evidence-Kit/evidence
cp /mnt/d/AETHER-X/evidence/runtime_smoke_20261115/runtime_snapshot.txt \
  /mnt/d/AETHER-X/Release-Evidence-Kit/evidence/runtime_snapshot_20261115.txt
```

For a complete evidence handoff, also copy the actual `colcon` test result output/XML files and the full launch logs from `~/.ros/log/` into the evidence folder. Review logs for private paths or personal information before sharing them publicly.
