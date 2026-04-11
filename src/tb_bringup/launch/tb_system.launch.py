#!/usr/bin/env python3
#
# Copyright 2026 RAS lab, FSU.
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
#
# Authors: Sridhar Babu Mudhangulla

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    namespace = "tb1"

    # -------------------------------
    # TurtleBot bringup (include)
    # -------------------------------
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={'namespace': namespace}.items(),
    )

    # -------------------------------
    # Velocity smoother config
    # -------------------------------
    config_file = os.path.join(
        get_package_share_directory('tb_bringup'), 'config', 'tb_velocity_smoother.yaml'
    )

    # -------------------------------
    # Watchdog
    # -------------------------------
    watchdog_node = Node(
        package="tb_watchdog",
        executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog",
        parameters=[
            {
                "input_topic": f"/{namespace}/cmd_vel_raw",
                "output_topic": f"/{namespace}/cmd_vel_watchdog",
                "timeout_sec": 0.5,
                "publish_rate": 20.0,
            }
        ],
    )

    # -------------------------------
    # Velocity smoother
    # -------------------------------
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[config_file],
        remappings=[
            ("cmd_vel", f"/{namespace}/cmd_vel_watchdog"),
            ("cmd_vel_smoothed", f"/{namespace}/cmd_vel"),
        ],
    )

    # -------------------------------
    # Lifecycle activation
    # -------------------------------
    configure_node = TimerAction(
        period = 8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', f'/{namespace}/velocity_smoother', 'configure'],
                output='screen'
            )
        ]
    )
    activate_node = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', f'/{namespace}/velocity_smoother', 'activate'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        turtlebot_launch,
        watchdog_node,
        velocity_smoother_node,
        configure_node,
        activate_node,
    ])
