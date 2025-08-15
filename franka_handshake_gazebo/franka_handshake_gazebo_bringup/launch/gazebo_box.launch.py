# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node

def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_handshake_description'),
        'urdf',
        'franka_handshake.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]



def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_handshake_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(
        get_package_share_directory('franka_handshake_gazebo_bringup'),
        'worlds',
        'empty_w_wrench.sdf'
    )
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r', }.items(),
    )

    # ros2 run ros_gz_bridge parameter_bridge /world/empty/wrench/persistent@geometry_msgs/msg/Wrench@ignition.msgs.Wrench
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench]ignition.msgs.EntityWrench'
        ]
    )


    return LaunchDescription([
        gazebo_empty_world,
        parameter_bridge
    ])
