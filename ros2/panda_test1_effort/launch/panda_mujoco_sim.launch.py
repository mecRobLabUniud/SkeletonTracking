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

"""MuJoCo Panda simulation with the test1 effort trajectory sender.

By default only the MuJoCo simulation is started.  With ``real_robot:=true``
the real Panda is brought up as well (in its own ROS namespace, default
``real``) and a second sender executes the same trajectory on the hardware::

    # Simulation only (default)
    ros2 launch panda_test1_effort panda_mujoco_sim.launch.py

    # Simulation + real robot, safe dry run (builds/logs, sends nothing)
    ros2 launch panda_test1_effort panda_mujoco_sim.launch.py \
        real_robot:=true robot_ip:=172.16.0.2 real_dry_run:=true

    # Simulation + real robot, half speed
    ros2 launch panda_test1_effort panda_mujoco_sim.launch.py \
        real_robot:=true robot_ip:=172.16.0.2 real_dry_run:=false \
        real_time_scale:=2.0

    # Simulation + real robot, manual trigger only
    ros2 launch panda_test1_effort panda_mujoco_sim.launch.py \
        real_robot:=true robot_ip:=172.16.0.2 real_dry_run:=false \
        real_auto_start:=false

The real robot is placed in the ``real`` namespace, so its controller manager
is ``/real/controller_manager`` and its action server is
``/real/panda_effort_jtc/follow_joint_trajectory``.  This keeps it from
colliding with the MuJoCo controller manager (``/controller_manager``) and
with the simulation's ``/panda_effort_jtc`` action server.

Note: the MuJoCo controller manager does not answer service calls when the
simulation runs with ``headless:=true render_offscreen:=false``.  Keep the
default rendering settings (or use ``render_offscreen:=true``) when you need
the controllers to start.

Safety: on hardware the trajectory is executed in joint torque mode.  Enable
the FCI, keep the enabling device in reach, clear the workspace, and put the
robot close to the test1 start pose
``[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`` (the sender refuses to move if the
robot is farther than ``max_approach_distance`` from it).  ``real_dry_run``
defaults to ``true`` on purpose; real motion requires an explicit
``real_dry_run:=false``.
"""

import os
import tempfile

import yaml

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
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _namespaced_param_file(source_file, namespace):
    """Copy a ros2_control parameter file, scoping every node to ``namespace``.

    ``franka_bringup`` ships ``single_controllers.yaml`` with relative node
    names (``controller_manager:``, ``joint_state_broadcaster:`` ...).  ROS 2
    only matches those nodes when they live in the root namespace, so running
    the real stack inside a namespace silently drops all of their parameters.
    This helper rewrites the node names to ``/<namespace>/<node>`` and returns
    the path of the rewritten copy.
    """
    with open(source_file, 'r') as stream:
        data = yaml.safe_load(stream) or {}

    prefix = '/' + namespace.strip('/')
    scoped = {}
    for node_name, node_params in data.items():
        scoped[node_name if node_name.startswith('/') else prefix + '/' + node_name] = (
            node_params
        )

    output_file = os.path.join(
        tempfile.gettempdir(),
        'panda_real_{}_{}'.format(
            namespace.strip('/') or 'root', os.path.basename(source_file)),
    )
    with open(output_file, 'w') as stream:
        yaml.safe_dump(scoped, stream, sort_keys=False)
    return output_file


def _real_robot_group(context):
    """Build the optional real-robot part of the launch description."""
    real_namespace = LaunchConfiguration('real_namespace').perform(context).strip('/')
    robot_ip = LaunchConfiguration('robot_ip')
    real_load_gripper = LaunchConfiguration('real_load_gripper')
    real_use_fake_hardware = LaunchConfiguration('real_use_fake_hardware')
    real_use_rviz = LaunchConfiguration('real_use_rviz')
    real_auto_start = LaunchConfiguration('real_auto_start')
    real_dry_run = LaunchConfiguration('real_dry_run')
    real_time_scale = LaunchConfiguration('real_time_scale')
    trajectory_dir = LaunchConfiguration('trajectory_dir')
    trajectory_file = LaunchConfiguration('trajectory_file')
    approach_duration = LaunchConfiguration('approach_duration')
    max_approach_distance = LaunchConfiguration('max_approach_distance')
    start_delay = LaunchConfiguration('start_delay')

    franka_description = get_package_share_directory('franka_description')
    franka_bringup = get_package_share_directory('franka_bringup')
    pkg_share = get_package_share_directory('panda_test1_effort')

    real_robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_description, 'robots', 'real', 'panda_arm.urdf.xacro'),
        ' hand:=', real_load_gripper,
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=', real_use_fake_hardware,
        ' fake_sensor_commands:=false',
    ])
    real_controller_config = _namespaced_param_file(
        os.path.join(franka_bringup, 'config', 'real', 'single_controllers.yaml'),
        real_namespace)
    effort_controller_config = _namespaced_param_file(
        os.path.join(pkg_share, 'config', 'panda_effort_jtc.yaml'),
        real_namespace)

    # Same nodes as franka_bringup/launch/real/franka.launch.py, but with the
    # parameters re-scoped to the namespace and with spawners that resolve
    # their controller manager relative to the namespace.
    return [
        GroupAction(
            actions=[
                PushRosNamespace(real_namespace),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': real_robot_description}],
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

                # controller_manager + Franka hardware interface.  Do not set
                # an explicit node name here: launch would pass `-r
                # __node:=...`, which would also rename every controller node
                # created inside this process.
                Node(
                    package='franka_control2',
                    executable='franka_control2_node',
                    output='screen',
                    parameters=[
                        {'robot_description': real_robot_description},
                        real_controller_config,
                    ],
                    remappings=[('joint_states', 'franka/joint_states')],
                    on_exit=Shutdown(),
                ),

                # 'controller_manager' is relative on purpose: the spawner
                # prefixes it with the spawner namespace ('/real').
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
                    condition=UnlessCondition(real_use_fake_hardware),
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
                        'use_fake_hardware': real_use_fake_hardware,
                    }.items(),
                    condition=IfCondition(real_load_gripper),
                ),

                # Effort-based joint trajectory controller (same config as sim).
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

                # Second sender, publishing into /real.
                Node(
                    package='panda_test1_effort',
                    executable='test1_trajectory_sender',
                    name='test1_trajectory_sender',
                    output='screen',
                    parameters=[{
                        'trajectory_dir': trajectory_dir,
                        'trajectory_file': trajectory_file,
                        'auto_start': real_auto_start,
                        'dry_run': real_dry_run,
                        'time_scale': real_time_scale,
                        'approach_duration': approach_duration,
                        'max_approach_distance': max_approach_distance,
                        'start_delay': start_delay,
                        'joint_state_topic': PythonExpression(
                            ["'/", real_namespace, "/joint_states'"]),
                        'controller_action': PythonExpression(
                            ["'/", real_namespace,
                             "/panda_effort_jtc/follow_joint_trajectory'"]),
                    }],
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['--display-config', os.path.join(
                        franka_description, 'rviz', 'visualize_franka.rviz')],
                    condition=IfCondition(real_use_rviz),
                ),
            ],
        )
    ]


def generate_launch_description():
    # Simulation
    trajectory_dir = LaunchConfiguration('trajectory_dir')
    trajectory_file = LaunchConfiguration('trajectory_file')
    auto_start = LaunchConfiguration('auto_start')
    dry_run = LaunchConfiguration('dry_run')
    time_scale = LaunchConfiguration('time_scale')
    approach_duration = LaunchConfiguration('approach_duration')
    max_approach_distance = LaunchConfiguration('max_approach_distance')
    start_delay = LaunchConfiguration('start_delay')
    headless = LaunchConfiguration('headless')
    render_offscreen = LaunchConfiguration('render_offscreen')
    unpause = LaunchConfiguration('unpause')
    real_robot = LaunchConfiguration('real_robot')

    # The MuJoCo scene always contains the Panda hand (see franka_sim.launch.py).
    arm_id = 'panda'
    load_gripper = True
    initial_positions = '0.0 -0.785 0.0 -2.356 0.0 1.571 0.785'

    franka_description = get_package_share_directory('franka_description')
    franka_bringup = get_package_share_directory('franka_bringup')
    pkg_share = get_package_share_directory('panda_test1_effort')

    robot_xacro = os.path.join(
        franka_description, 'robots', 'sim', 'panda_arm_sim.urdf.xacro')
    scene_file = 'scene.xml' if load_gripper else 'scene_ng.xml'
    mujoco_model = os.path.join(
        franka_description, 'mujoco', 'franka', scene_file)
    mjros_config = os.path.join(
        franka_bringup, 'config', 'sim', 'single_sim_controllers.yaml')
    controller_config = os.path.join(
        pkg_share, 'config', 'panda_effort_jtc.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', robot_xacro,
        ' arm_id:=', arm_id,
        ' hand:=', str(load_gripper).lower(),
        ' initial_positions:=', '"' + initial_positions + '"',
    ])

    # ── MuJoCo simulation ────────────────────────────────────────────────────
    mujoco_server = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(franka_bringup, 'launch', 'sim',
                         'launch_mujoco_ros_server.launch')),
        launch_arguments={
            'use_sim_time': 'true',
            'modelfile': mujoco_model,
            'verbose': 'false',
            'ns': '',
            'unpause': unpause,
            'headless': headless,
            'render_offscreen': render_offscreen,
            'mujoco_plugin_config': mjros_config,
        }.items(),
    )

    sim_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    sim_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    sim_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_effort_jtc',
                   '-t', 'joint_trajectory_controller/JointTrajectoryController',
                   '-c', '/controller_manager',
                   '-p', controller_config],
        output='screen',
    )

    sim_trajectory_sender = Node(
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
        # ── Simulation arguments ─────────────────────────────────────────────
        DeclareLaunchArgument(
            'trajectory_dir',
            default_value='/home/lab/SkeletonTracking/src/trajectories/test1',
            description='Directory containing q.csv (and optional qd/qdd.csv).'),
        DeclareLaunchArgument(
            'trajectory_file', default_value='q.csv',
            description='Joint-position CSV relative to trajectory_dir.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Simulation: send the trajectory automatically.'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='Simulation: build/log the trajectory without sending.'),
        DeclareLaunchArgument(
            'time_scale', default_value='1.0',
            description='Simulation: slow-motion factor (2.0 = half speed).'),
        DeclareLaunchArgument(
            'approach_duration', default_value='3.0',
            description='Duration of the smooth move to the first sample (s).'),
        DeclareLaunchArgument(
            'max_approach_distance', default_value='0.5',
            description='Refuse to run if farther than this from the start (rad).'),
        DeclareLaunchArgument(
            'start_delay', default_value='2.0',
            description='Wait after the first joint state before sending (s).'),
        DeclareLaunchArgument(
            'unpause', default_value='true',
            description='Unpause the simulation on start.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run MuJoCo without a viewer window.  Keep '
                        'render_offscreen:=true when headless.'),
        DeclareLaunchArgument(
            'render_offscreen', default_value='true',
            description='Enable offscreen rendering.'),

        # ── Real robot arguments ─────────────────────────────────────────────
        DeclareLaunchArgument(
            'real_robot', default_value='false',
            description='Also bring up the real Panda and move it.'),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.2',
            description='FCI IP address or hostname of the real Panda.'),
        DeclareLaunchArgument(
            'real_namespace', default_value='real',
            description='ROS namespace used for the real robot stack '
                        '(a plain name, without a leading slash).'),
        DeclareLaunchArgument(
            'real_auto_start', default_value='true',
            description='Real robot: send the trajectory automatically.'),
        DeclareLaunchArgument(
            'real_dry_run', default_value='true',
            description='Real robot: true = log only, false = actually move.'),
        DeclareLaunchArgument(
            'real_time_scale', default_value='2.0',
            description='Real robot: slow-motion factor (2.0 = half speed).'),
        DeclareLaunchArgument(
            'real_load_gripper', default_value='false',
            description='Real robot: load the Franka Hand.'),
        DeclareLaunchArgument(
            'real_use_fake_hardware', default_value='false',
            description='Real robot: use fake hardware instead of the Panda.'),
        DeclareLaunchArgument(
            'real_use_rviz', default_value='false',
            description='Real robot: start RViz.'),

        # ── Simulation nodes ─────────────────────────────────────────────────
        mujoco_server,
        sim_robot_state_publisher,
        sim_joint_state_broadcaster,
        sim_effort_controller,
        sim_trajectory_sender,

        # ── Optional real robot nodes ────────────────────────────────────────
        OpaqueFunction(
            function=_real_robot_group,
            condition=IfCondition(real_robot),
        ),
    ])
