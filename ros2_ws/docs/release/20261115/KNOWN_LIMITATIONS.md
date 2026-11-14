# AETHER-X — Known Limitations and Claim Boundaries

This file is intended to prevent accidental overstatement in a CV, portfolio, demo, or interview.

| Area | Evidence-supported wording | Do not claim |
|---|---|---|
| Hardware | ROS 2 mock hardware simulation using `mock_components/GenericSystem` | Tested on physical robot / production hardware |
| Timing | Configured controller manager update rate 100 Hz; sensor streams measured at 100/10/10 Hz in WSL2 | Hard real-time guarantees or deterministic RT scheduling |
| Control | Differential-drive controller and four simulated wheel joints active | Hardware actuator qualification |
| Localization | EKF and GNSS transform topics publishing in simulation | Field-tested GNSS/IMU accuracy |
| Mapping | SLAM Toolbox active and `/map` published at 5 cm resolution | Complete or metrically certified mapping of an entire environment |
| Navigation | Path planning and a NavigateToPose goal succeeded in simulation; zero command output verified afterward | Generalized navigation success in arbitrary environments |
| Static obstacles | Earlier simulated static-obstacle route succeeded | All obstacle configurations handled |
| Dynamic obstacles | Runtime obstacle injection affected costmaps; test goal was rejected with no valid path | Successful dynamic replanning while moving |
| Test suite | 86 reported tests, 0 failures/errors, 8 skipped | “All tests passed without skips” |

## Open engineering items

- Inspect and explain all eight skipped tests.
- Resolve or document why ros2_control command-limit enforcement is disabled.
- Decide whether to enable/configure a real-time-capable host outside WSL2 if hard RT performance is a project requirement.
- Design a controlled dynamic-obstacle scenario with a known feasible alternate route and validate replanning during motion.
- Define measurable mapping coverage and localization error criteria before making quantitative accuracy claims.
