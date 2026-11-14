# Copyright 2026 Dhanusri Veerappan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Licensed under the Apache License, Version 2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory(
        'aether_description'
    )

    control_share = get_package_share_directory(
        'aether_control'
    )

    control_launch_file = os.path.join(
        control_share,
        'launch',
        'control.launch.py'
    )

    rviz_config = os.path.join(
        description_share,
        'rviz',
        'aether_x.rviz'
    )

    start_rviz = LaunchConfiguration('rviz')

    control_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file)
    )

    core_publisher = Node(
        package='aether_core',
        executable='aether_node',
        name='aether_node',
        output='screen'
    )

    core_monitor = Node(
        package='aether_core',
        executable='aether_monitor',
        name='aether_monitor',
        output='screen'
    )

    imu_simulator = Node(
        package='aether_sensors',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen'
    )

    gnss_simulator = Node(
        package='aether_sensors',
        executable='gnss_simulator',
        name='gnss_simulator',
        output='screen'
    )

    lidar_simulator = Node(
        package='aether_sensors',
        executable='lidar_simulator',
        name='lidar_simulator',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 visualization'
        ),
        control_system,
        core_publisher,
        core_monitor,
        imu_simulator,
        gnss_simulator,
        lidar_simulator,
        rviz_node
    ])
