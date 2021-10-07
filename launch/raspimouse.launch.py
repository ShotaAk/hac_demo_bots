# Copyright 2021 ShotaAk
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

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('raspimouse_description'),
        'urdf',
        'raspimouse.urdf.xacro')
    params = {'robot_description': Command(['xacro ', xacro_file, 
                                            ' gazebo_plugin:=true',
                                            ' config_file_package:=hac_demo_bots',
                                            ' config_file_path:=config/gazebo_controllers.yaml'])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    declare_challenge_number = DeclareLaunchArgument(
        'challenge', default_value='01',
        description=('Set challenge number: 00 ~ 08')
    )

    field = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('hac_gazebo'),
                '/launch/challenge', LaunchConfiguration('challenge'), '.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'raspimouse',
                                   '-x', '0', '-y', '0', '-z', '0.3',
                                   '-topic', '/robot_description'],
                        output='screen')

    config_file_path = os.path.join(get_package_share_directory('hac_demo_bots'), 'config', 'ps3.config.yaml')
    print(config_file_path)

    joy_node = Node(
            package='joy', executable='joy_node', name='joy_node',
            # namespace='diff_drive_controller',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    teleop_node = Node(
            package='teleop_twist_joy', executable='teleop_node',
            # namespace='diff_drive_controller',
            remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
            name='teleop_twist_joy_node', parameters=[config_file_path])

    joint_state_controller = ExecuteProcess(
      cmd=['ros2 run controller_manager spawner.py joint_state_controller'],
      shell=True,
      output='screen'
    )

    diff_drive_controller = ExecuteProcess(
      cmd=['ros2 run controller_manager spawner.py diff_drive_controller'],
      shell=True,
      output='screen'
    )

    recognition_node = Node(
            package='hac_demo_bots',
            executable='hac_recognition.py',
            name='recognition_node',
            remappings=[('image_raw', 'camera/image_raw')],
            output='screen'
            # parameters=[config_file_path]
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_controller,
                on_exit=[diff_drive_controller],
            )
        ),
        declare_challenge_number,
        field,
        node_robot_state_publisher,
        spawn_entity,
        joy_node,
        teleop_node,
        # recognition_node
    ])
