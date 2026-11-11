# Copyright 2026 SkeletonTracking
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

"""Bring up a real Panda, the effort JTC and the test1 trajectory sender.

Examples::

    # Safe dry run: build/log the trajectory, send nothing.
    ros2 launch panda_test1_effort panda_test1_effort.launch.py \
        robot_ip:=172.16.0.2 dry_run:=true

    # First real motion: half speed.
    ros2 launch panda_test1_effort panda_test1_effort.launch.py \
        robot_ip:=172.16.0.2 dry_run:=false time_scale:=2.0

    # Real-time motion.
    ros2 launch panda_test1_effort panda_test1_effort.launch.py \
        robot_ip:=172.16.0.2 dry_run:=false time_scale:=1.0

Safety: enable the FCI, keep the enabling device in reach, clear the
workspace, and put the robot close to the test1 start pose
``[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`` (the sender refuses to move if the
robot is more than ``max_approach_distance`` away from it).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    use_rviz = LaunchConfiguration('use_rviz')

    trajectory_dir = LaunchConfiguration('trajectory_dir')
    trajectory_file = LaunchConfiguration('trajectory_file')
    auto_start = LaunchConfiguration('auto_start')
    dry_run = LaunchConfiguration('dry_run')
    time_scale = LaunchConfiguration('time_scale')
    approach_duration = LaunchConfiguration('approach_duration')
    max_approach_distance = LaunchConfiguration('max_approach_distance')
    start_delay = LaunchConfiguration('start_delay')

    franka_bringup = get_package_share_directory('franka_bringup')
    pkg_share = get_package_share_directory('panda_test1_effort')
    controller_config = os.path.join(
        pkg_share, 'config', 'panda_effort_jtc.yaml')

    # Real robot bringup: robot_state_publisher, joint_state_publisher,
    # franka_control2 (controller_manager + hardware interface),
    # joint_state_broadcaster, franka_robot_state_broadcaster and the gripper.
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_bringup, 'launch', 'real', 'franka.launch.py')),
        launch_arguments={
            'robot_ip': robot_ip,
            'load_gripper': load_gripper,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'use_rviz': use_rviz,
        }.items(),
    )

    # Effort-based joint trajectory controller.
    effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_effort_jtc',
                   '-t', 'joint_trajectory_controller/JointTrajectoryController',
                   '-c', '/controller_manager',
                   '-p', controller_config],
        output='screen',
    )

    # The test1 trajectory sender.  auto_start waits for a complete
    # /joint_states message and for the controller action server, then sends.
    trajectory_sender = Node(
        package='panda_test1_effort',
        executable='test1_trajectory_sender',
        name='test1_trajectory_sender',
        output='screen',
        parameters=[{
            'trajectory_dir': trajectory_dir,
            'trajectory_file': trajectory_file,
            'auto_start': auto_start,
            'dry_run': dry_run,
            'time_scale': time_scale,
            'approach_duration': approach_duration,
            'max_approach_distance': max_approach_distance,
            'start_delay': start_delay,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.2',
            description='FCI IP address or hostname of the Panda.'),
        DeclareLaunchArgument(
            'load_gripper', default_value='false',
            description='Load the Franka Hand.'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Use fake_components instead of a real robot.'),
        DeclareLaunchArgument(
            'fake_sensor_commands', default_value='false',
            description='Only used with use_fake_hardware:=true.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Start RViz.'),
        DeclareLaunchArgument(
            'trajectory_dir',
            default_value='/home/lab/SkeletonTracking/src/trajectories/test1',
            description='Directory containing q.csv (and optional qd/qdd.csv).'),
        DeclareLaunchArgument(
            'trajectory_file', default_value='q.csv',
            description='Joint-position CSV relative to trajectory_dir.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Send automatically once the robot is ready.'),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Build/log the trajectory but do not move the robot.'),
        DeclareLaunchArgument(
            'time_scale', default_value='1.0',
            description='Slow-motion factor (2.0 = half speed).'),
        DeclareLaunchArgument(
            'approach_duration', default_value='3.0',
            description='Duration of the smooth move to the first sample (s).'),
        DeclareLaunchArgument(
            'max_approach_distance', default_value='0.5',
            description='Refuse to run if farther than this from the start (rad).'),
        DeclareLaunchArgument(
            'start_delay', default_value='2.0',
            description='Wait after the first joint state before sending (s).'),

        robot_bringup,
        effort_controller,
        trajectory_sender,
    ])
