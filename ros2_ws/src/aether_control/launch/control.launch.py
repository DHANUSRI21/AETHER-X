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
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory(
        'aether_description'
    )

    control_share = get_package_share_directory(
        'aether_control'
    )

    description_launch = os.path.join(
        description_share,
        'launch',
        'display.launch.py'
    )

    controller_config = os.path.join(
        control_share,
        'config',
        'controllers.yaml'
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[controller_config]
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        output='screen'
    )

    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[diff_drive_spawner]
        )
    )

    return LaunchDescription([
        robot_description,
        controller_manager,
        load_controllers,
        TimerAction(
            period=3.0,
            actions=[joint_state_spawner]
        )
    ])
