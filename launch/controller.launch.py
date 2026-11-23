"""Bring up the real Franka Panda so that the SSM pipeline can drive it.

This launches every node that the real robot needs in order to move through the
``joint_trajectory_controller`` action interface:

  * robot_state_publisher with the real Panda description (FCI IP, optional
    Franka Hand)
  * joint_state_publisher, aggregating ``franka/joint_states`` (and the gripper)
    into ``/joint_states``
  * franka_control2_node: controller_manager + Franka hardware interface
  * joint_state_broadcaster (always) and franka_robot_state_broadcaster
    (hardware only)
  * panda_effort_jtc: effort-based JointTrajectoryController used by the
    SSM/``robot_controller_ros2`` pipeline
  * optional Franka Hand driver
  * optional robot_controller_ros2, which receives the ``ROBOT_12`` stream and
    either follows it live (``streaming:=true``, the default) or sends the
    accumulated trajectory to the controller above when the stream ends

Everything runs in the root namespace, so the action server is
``/panda_effort_jtc/follow_joint_trajectory`` and the joint states are
``/joint_states``.

Usage::

    source /opt/ros/humble/setup.bash
    source /home/lab/multipanda_ws/install/setup.bash

    # Hardware stack only (no motion)
    ros2 launch /home/lab/SkeletonTracking/launch/real_robot.launch.py

    # Hardware stack + controller: the arm moves to the start pose as soon as
    # the controller is up, then follows the live stream
    ros2 launch /home/lab/SkeletonTracking/launch/real_robot.launch.py \
        start_controller:=true traj_index:=3

If the robot is not already at the trajectory start pose, measure the largest
single-joint offset and raise ``max_approach_distance`` to cover it, e.g.::

    ros2 launch /home/lab/SkeletonTracking/launch/real_robot.launch.py \
        start_controller:=true traj_index:=3 \
        start_duration:=8.0 max_approach_distance:=2.0

The approach is a quintic joint-space move with zero velocity/acceleration at
both ends, so a longer ``approach_duration`` makes the move gentler.

The ``ROBOT_12`` stream itself comes from the SSM pipeline, typically::

    ./run.sh --stream --test 7 --robot --traj 3 --core SSM

While the person stays inside the robot workspace, ``main_SSM`` keeps inserting
safety stops and never ends the stream.  With ``streaming:=true`` (the default)
the controller does not wait for the end of the stream: about one second after
the first ``ROBOT_12`` messages it sends the first goal (approach + one chunk)
and then keeps replacing the active goal with the next chunk, so the robot
follows the person live.  ``streaming:=false`` restores the accumulate and
send-at-the-end behaviour, in which case the trajectory is only sent when the
stream ends (``stream_timeout`` elapsed) or when ``~/send`` is called::

    ros2 service call /robot_controller_ros2/send std_srvs/srv/Trigger "{}"

In streaming mode ``~/cancel`` stops the streaming and cancels the active goal::

    ros2 service call /robot_controller_ros2/cancel std_srvs/srv/Trigger "{}"

The robot lags the stream by roughly ``approach_duration + chunk_duration +
chunk_lead`` seconds; move the robot close to the trajectory start pose (and
lower ``approach_duration``) to reduce it.

Safety: keep the enabling device (E-stop) in reach, clear the workspace and
place the robot close to the trajectory start pose before launching
with ``start_controller:=true``.  The controller refuses to move if the robot is farther
than ``max_approach_distance`` from the first trajectory point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _real_robot_group(context):
    """Build the real-robot nodes, resolved against the launch arguments."""
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_rviz = LaunchConfiguration('use_rviz')

    franka_description = get_package_share_directory('franka_description')
    franka_bringup = get_package_share_directory('franka_bringup')
    pkg_share = get_package_share_directory('panda_test1_effort')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_description, 'robots', 'real', 'panda_arm.urdf.xacro'),
        ' hand:=', load_gripper,
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=false',
    ])
    controller_config = os.path.join(
        franka_bringup, 'config', 'real', 'single_controllers.yaml')
    effort_controller_config = os.path.join(
        pkg_share, 'config', 'panda_effort_jtc.yaml')

    return [
        GroupAction(
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description}],
                ),
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen',
                    parameters=[{
                        'source_list': ['franka/joint_states',
                                        'panda_gripper/joint_states'],
                        'rate': 30,
                    }],
                ),

                # controller_manager + Franka hardware interface.  Do not set an
                # explicit node name: launch would also rename every controller
                # node created inside this process.
                Node(
                    package='franka_control2',
                    executable='franka_control2_node',
                    output='screen',
                    parameters=[
                        {'robot_description': robot_description},
                        controller_config,
                    ],
                    remappings=[('joint_states', 'franka/joint_states')],
                    on_exit=Shutdown(),
                ),

                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '-c', 'controller_manager'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['franka_robot_state_broadcaster',
                               '-c', 'controller_manager'],
                    condition=UnlessCondition(use_fake_hardware),
                    output='screen',
                ),

                # Effort-based JointTrajectoryController (same config as sim).
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['panda_effort_jtc',
                               '-t',
                               'joint_trajectory_controller/JointTrajectoryController',
                               '-c', 'controller_manager',
                               '-p', effort_controller_config],
                    output='screen',
                ),

                # Franka Hand.
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('franka_gripper'),
                            'launch', 'gripper.launch.py',
                        ])
                    ]),
                    launch_arguments={
                        'robot_ip': robot_ip,
                        'use_fake_hardware': use_fake_hardware,
                    }.items(),
                    condition=IfCondition(load_gripper),
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['--display-config', os.path.join(
                        franka_description, 'rviz', 'visualize_franka.rviz')],
                    condition=IfCondition(use_rviz),
                ),
            ]
        ),
    ] + _optional_controller(context)


def _optional_controller(context):
    """Conditionally start robot_controller_ros2.

    The ROS2 port of robot_controller.cpp catches the ``ROBOT_12`` stream
    published by main_SSM and sends the accumulated trajectory to the effort
    JointTrajectoryController started by ``_real_robot_group``.
    """
    from launch.actions import ExecuteProcess

    executable = LaunchConfiguration('controller_executable').perform(context)
    repo_dir = LaunchConfiguration('repo_dir').perform(context)
    traj_index = LaunchConfiguration('traj_index').perform(context)
    start_duration = LaunchConfiguration('start_duration').perform(context)
    stream_timeout = LaunchConfiguration('stream_timeout').perform(context)
    approach_duration = LaunchConfiguration('approach_duration').perform(context)
    max_approach_distance = LaunchConfiguration('max_approach_distance').perform(context)
    start_tolerance = LaunchConfiguration('start_tolerance').perform(context)
    streaming = LaunchConfiguration('streaming').perform(context)
    chunk_duration = LaunchConfiguration('chunk_duration').perform(context)
    chunk_lead = LaunchConfiguration('chunk_lead').perform(context)
    chunk_ramp = LaunchConfiguration('chunk_ramp').perform(context)
    min_chunk_duration = LaunchConfiguration('min_chunk_duration').perform(context)
    max_chunk_step = LaunchConfiguration('max_chunk_step').perform(context)
    max_ramp_velocity = LaunchConfiguration('max_ramp_velocity').perform(context)

    controller = ExecuteProcess(
        cmd=[
            executable, traj_index, repo_dir,
            '--ros-args',
            '-p', 'start_duration:=' + start_duration,
            '-p', 'controller_action:=/panda_effort_jtc/follow_joint_trajectory',
            '-p', 'joint_state_topic:=/joint_states',
            '-p', 'stream_timeout:=' + stream_timeout,
            '-p', 'approach_duration:=' + approach_duration,
            '-p', 'max_approach_distance:=' + max_approach_distance,
            '-p', 'start_tolerance:=' + start_tolerance,
            '-p', 'streaming:=' + streaming,
            '-p', 'chunk_duration:=' + chunk_duration,
            '-p', 'chunk_lead:=' + chunk_lead,
            '-p', 'chunk_ramp:=' + chunk_ramp,
            '-p', 'min_chunk_duration:=' + min_chunk_duration,
            '-p', 'max_chunk_step:=' + max_chunk_step,
            '-p', 'max_ramp_velocity:=' + max_ramp_velocity,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controller')),
    )

    # No start-up timer: the controller waits by itself for the joint states
    # and the action server, then moves to the start pose right away.
    return [controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='172.16.0.2',
            description='FCI IP address or hostname of the real Panda.'),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='Also start the Franka Hand driver.'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware instead of the real Panda.'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz.'),

        # Optional robot_controller_ros2
        DeclareLaunchArgument(
            'start_controller',
            default_value='false',
            description='Also start robot_controller_ros2 (expects the '
                        'ROBOT_12 stream from main_SSM).'),
        DeclareLaunchArgument(
            'traj_index',
            default_value='1',
            description='Trajectory index passed to robot_controller_ros2.'),
        DeclareLaunchArgument(
            'start_duration',
            default_value='3.0',
            description='Duration of the initial move to the first '
                        'trajectory point (s).'),
        DeclareLaunchArgument(
            'stream_timeout',
            default_value='5.0',
            description='Seconds without ROBOT_12 before the stream is '
                        'considered finished.'),
        DeclareLaunchArgument(
            'approach_duration',
            default_value='3.0',
            description='Duration of the smooth move to the first point (s).'),
        DeclareLaunchArgument(
            'max_approach_distance',
            default_value='0.5',
            description='Refuse to move if farther than this from the first '
                        'trajectory point (rad).'),
        DeclareLaunchArgument(
            'start_tolerance',
            default_value='0.15',
            description='Start-pose tolerance when approach_duration is 0.'),
        DeclareLaunchArgument(
            'streaming',
            default_value='true',
            description='Follow the ROBOT_12 stream live instead of sending '
                        'one trajectory when the stream ends.'),
        DeclareLaunchArgument(
            'chunk_duration',
            default_value='1.0',
            description='Trajectory length played by each streamed goal (s).'),
        DeclareLaunchArgument(
            'chunk_lead',
            default_value='0.3',
            description='Send the next chunk this long before the current one '
                        'ends (s).'),
        DeclareLaunchArgument(
            'chunk_ramp',
            default_value='0.2',
            description='Blend from the measured pose into each chunk (s).'),
        DeclareLaunchArgument(
            'min_chunk_duration',
            default_value='0.3',
            description='Smallest amount of new data worth a goal (s).'),
        DeclareLaunchArgument(
            'max_chunk_step',
            default_value='0.35',
            description='Refuse a chunk whose first move is larger (rad).'),
        DeclareLaunchArgument(
            'max_ramp_velocity',
            default_value='1.0',
            description='Stretch the blend if it would be faster (rad/s).'),
        DeclareLaunchArgument(
            'controller_executable',
            default_value='/home/lab/SkeletonTracking/build/new_robot_controller_ros2',
            description='Absolute path of the robot_controller_ros2 binary.'),
        DeclareLaunchArgument(
            'repo_dir',
            default_value='/home/lab/SkeletonTracking',
            description='Repository root (URDF + trajectories).'),

        OpaqueFunction(function=_real_robot_group),
    ])