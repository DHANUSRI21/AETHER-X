# AETHER-X — Architecture Overview

## System boundary

AETHER-X is a ROS 2 Jazzy simulation-first autonomous mobile robot stack. Its current runtime uses `mock_components/GenericSystem`, simulated sensor publishers, robot_localization, SLAM Toolbox, and Nav2.

## Package responsibilities

- **aether_description:** Xacro/URDF, robot geometry and sensor frames, ros2_control interfaces, RViz model setup.
- **aether_control:** ros2_control manager configuration, joint-state broadcaster and differential-drive controller.
- **aether_core:** core heartbeat/status publisher and system state monitor.
- **aether_sensors:** simulated motion-aware IMU, GNSS, and 360-beam LiDAR streams.
- **aether_localization:** local/global EKF, GNSS-to-odometry integration, SLAM Toolbox configuration and launch.
- **aether_navigation:** Nav2 planner/controller/BT navigator/behavior servers, costmaps and launch configuration.
- **aether_bringup:** composition of base description/control/core/sensor processes, with optional RViz.

## Main data paths

1. `ros2_control_node` loads the Xacro robot description and mock hardware.
2. `joint_state_broadcaster` publishes `/joint_states`.
3. `diff_drive_controller` accepts velocity commands on `/diff_drive_controller/cmd_vel`, publishes `/diff_drive_controller/odom`, and owns `odom -> base_footprint`.
4. IMU, GNSS and LiDAR simulators consume wheel odometry for simulated motion response.
5. The local EKF fuses wheel odometry and IMU yaw rate. GNSS is converted through `navsat_transform` and fed into the global localization path.
6. SLAM Toolbox consumes `/aether/sensors/lidar`, publishes `/map`, and owns `map -> odom`.
7. Nav2 consumes the map, localization transforms and LiDAR-backed costmaps, then sends velocity commands to the differential-drive controller.

## Important ownership rule

Avoid multiple publishers for the same transform. In the current design, the controller owns `odom -> base_footprint` and SLAM owns `map -> odom`. The global EKF is configured with TF publication disabled to avoid a competing transform publisher.

## Operational sequence

1. `ros2 launch aether_bringup bringup.launch.py`
2. `ros2 launch aether_localization localization.launch.py`
3. Configure and activate `/slam_toolbox` lifecycle.
4. `ros2 launch aether_navigation navigation.launch.py`
5. Verify Nav2 lifecycle state, action servers and costmap topics before sending a goal.

Start each launch once. Stop prior launch sessions with Ctrl+C before restarting to avoid duplicate nodes and ambiguous controller-manager service calls.
