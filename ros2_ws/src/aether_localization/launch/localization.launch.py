from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(
        get_package_share_directory("aether_localization")
    )

    ekf_config = package_dir / "config" / "ekf.yaml"
    global_config = package_dir / "config" / "ekf_global.yaml"
    navsat_config = package_dir / "config" / "navsat.yaml"
    slam_config = package_dir / "config" / "slam.yaml"

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(ekf_config)],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[str(global_config)],
            remappings=[
                ("odometry/filtered", "/odometry/global"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[str(navsat_config)],
            remappings=[
                ("/gps/fix", "/aether/sensors/gnss"),
                ("/imu/data", "/aether/sensors/imu"),
            ],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(slam_config), {"use_sim_time": False}],
        ),
    ])
