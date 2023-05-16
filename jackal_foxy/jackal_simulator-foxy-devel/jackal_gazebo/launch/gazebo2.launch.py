# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ignition_ros2_control_demos_path = os.path.join(
        get_package_share_directory('ign_ros2_control_demos'))
    
    jackal_description_path = os.path.join(
        get_package_share_directory('jackal_description'))
    
    jackal_world_path = os.path.join(
        get_package_share_directory('jackal_gazebo'))

    xacro_file = os.path.join(jackal_description_path,
                              'urdf',
                              'jackal_lidar.urdf')
    
    world_file = os.path.join(jackal_world_path,
                              'worlds',
                              'jackal_track.sdf')
    
    # xacro_file = os.path.join(ignition_ros2_control_demos_path,
    #                           'urdf',
    #                           'test_diff_drive.xacro.urdf')
    
    print(xacro_file)
    doc = xacro.parse(open(xacro_file))
    print(doc)
    xacro.process_doc(doc)
    print("3")
    params = {'robot_description': doc.toxml()}

    print(params)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'Jackal',
                   '-allow_renaming', 'true'],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # keyboard_controller = ExecuteProcess(
    #     # cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #     #      'joint_state_broadcaster'],
    #     cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
    #           '--ros-args', '--remap', '/cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped'],
    #     output='screen'
    # )

    keyboard_controller_bridge = ExecuteProcess(
        # cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
        #      'joint_state_broadcaster'],
        cmd=['ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
              '/diff_drive_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    lidar_bridge = ExecuteProcess(
        # cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
        #      'joint_state_broadcaster'],
        cmd=['ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
              '/range@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan', '--ros-args', '-r', '/range:=/laser_scan'],
        output='screen'
    )



    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'diff_drive_base_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 4 ', world_file ])]),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('ros_ign_gazebo'),
        #                       'launch', 'ign_gazebo.launch.py')]),
        #     launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[keyboard_controller_bridge],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[lidar_bridge],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
