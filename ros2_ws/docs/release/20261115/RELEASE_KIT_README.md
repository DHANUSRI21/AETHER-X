# AETHER-X — Release Evidence Kit

This kit records the verified software and simulation validation state as of **15 November 2026**.

## Contents

- `docs/RUNTIME_VALIDATION_REPORT.md` — build, test and runtime results.
- `docs/ARCHITECTURE_OVERVIEW.md` — package roles and system data flow.
- `docs/KNOWN_LIMITATIONS.md` — claim boundaries and open engineering items.
- `docs/RELEASE_CHECKLIST.md` — completed and remaining release gates.
- `evidence/EVIDENCE_INDEX.md` — evidence register and instructions to add workstation logs.
- `evidence/screenshots/` — one supplementary RViz screenshot from 14 November 2026.

## Important

The live runtime snapshot was saved on the workstation at `/mnt/d/AETHER-X/evidence/runtime_smoke_20261115/runtime_snapshot.txt`. It is referenced in the evidence index but is not inside this ZIP because the Windows D: project workspace is not mounted in the artifact-building environment. Copy it into `evidence/` using the command in `EVIDENCE_INDEX.md`.

This is an evidence/documentation kit, not a copy of the full ROS 2 source workspace. The project was validated using mock ros2_control hardware in WSL2. See the report and limitations document before making portfolio or interview claims.
