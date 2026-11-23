"""Bring up the real Panda and execute src/trajectories/test3 with it.

This launch file starts:

  * every node of launch/real_robot.launch.py (robot_state_publisher,
    joint_state_publisher, franka_control2_node, the joint state broadcasters
    and the effort joint_trajectory_controller);
  * build_ros2/random_trajectory_controller, the small FollowJointTrajectory
    client from src/random_trajectory_controller.cpp.

The controller loads the trajectory set given by ``trajectory_dir`` (by
default ``<repository>/src/trajectories/test3``), solves the IK for its
Cartesian waypoints, samples the quintic point-to-point segments at 60 Hz,
prepends a smooth approach from the measured pose and sends one
FollowJointTrajectory goal to ``/panda_effort_jtc/follow_joint_trajectory``.

Usage::

    source /opt/ros/humble/setup.bash
    source /home/lab/multipanda_ws/install/setup.bash

    cmake -S /home/lab/SkeletonTracking -B /home/lab/SkeletonTracking/build_ros2 \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build /home/lab/SkeletonTracking/build_ros2 \
        --target random_trajectory_controller -j"$(nproc)"

    ros2 launch /home/lab/SkeletonTracking/launch/random_trajectory.launch.py

If the robot is not already at the first trajectory point, measure the largest
joint offset and raise ``max_approach_distance`` accordingly, for example::

    ros2 launch .../random_trajectory.launch.py \
        approach_duration:=8.0 max_approach_distance:=3.0

Use ``use_fake_hardware:=true`` to try the whole pipeline without moving a
real robot, and ``dry_run:=true`` to only build and print the goal.

Safety: keep the enabling device (E-stop) in reach, clear the workspace and
place the robot close to the trajectory start pose before launching.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# The launch file lives in <repository>/launch, so the defaults below work both
# on the real deployment (/home/lab/SkeletonTracking) and in any other checkout.
REPOSITORY_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_TRAJECTORY_DIR = os.path.join(REPOSITORY_DIR, "src", "trajectories", "test3")
DEFAULT_URDF = os.path.join(REPOSITORY_DIR, "src", "urdf", "panda.urdf")
DEFAULT_EXECUTABLE = os.path.join(
    REPOSITORY_DIR, "build", "random_trajectory_controller"
)
DEFAULT_REAL_ROBOT_LAUNCH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "real_robot.launch.py"
)


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    controller_executable = LaunchConfiguration("controller_executable")
    trajectory_dir = LaunchConfiguration("trajectory_dir")
    robot_urdf = LaunchConfiguration("robot_urdf")
    controller_action = LaunchConfiguration("controller_action")
    dry_run = LaunchConfiguration("dry_run")
    approach_duration = LaunchConfiguration("approach_duration")
    max_approach_distance = LaunchConfiguration("max_approach_distance")
    start_delay = LaunchConfiguration("start_delay")
    sample_rate = LaunchConfiguration("sample_rate")

    # Robot hardware + effort joint_trajectory_controller.  The controller is
    # started separately below, so the optional one of real_robot.launch.py is
    # explicitly disabled.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(DEFAULT_REAL_ROBOT_LAUNCH),
        launch_arguments={
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_fake_hardware": use_fake_hardware,
            "use_rviz": use_rviz,
            "start_controller": "false",
        }.items(),
    )

    # The one-shot trajectory client.  It waits by itself for the joint states
    # and the action server, so no start-up timer is needed.
    trajectory_controller = ExecuteProcess(
        cmd=[
            controller_executable,
            "--ros-args",
            "-p", ["trajectory_dir:=", trajectory_dir],
            "-p", ["robot_urdf:=", robot_urdf],
            "-p", ["controller_action:=", controller_action],
            "-p", ["joint_state_topic:=/joint_states"],
            "-p", ["sample_rate:=", sample_rate],
            "-p", ["approach_duration:=", approach_duration],
            "-p", ["max_approach_distance:=", max_approach_distance],
            "-p", ["start_delay:=", start_delay],
            "-p", ["dry_run:=", dry_run],
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.0.2",
            description="FCI IP address or hostname of the real Panda."),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="false",
            description="Also start the Franka Hand driver."),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use the ros2_control mock hardware instead of the robot."),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz."),

        DeclareLaunchArgument(
            "controller_executable",
            default_value=DEFAULT_EXECUTABLE,
            description="Absolute path of the random_trajectory_controller binary."),
        DeclareLaunchArgument(
            "trajectory_dir",
            default_value=DEFAULT_TRAJECTORY_DIR,
            description="Trajectory set directory (p_ref.csv/q_ref.csv + t_ref.csv)."),
        DeclareLaunchArgument(
            "robot_urdf",
            default_value=DEFAULT_URDF,
            description="Panda URDF used for the inverse kinematics."),
        DeclareLaunchArgument(
            "controller_action",
            default_value="/panda_effort_jtc/follow_joint_trajectory",
            description="FollowJointTrajectory action of the effort controller."),
        DeclareLaunchArgument(
            "sample_rate",
            default_value="60.0",
            description="Trajectory sampling rate (Hz)."),
        DeclareLaunchArgument(
            "approach_duration",
            default_value="3.0",
            description="Duration of the smooth move to the first point (s)."),
        DeclareLaunchArgument(
            "max_approach_distance",
            default_value="2.0",
            description="Refuse to move if farther than this from the first "
                        "trajectory point (rad)."),
        DeclareLaunchArgument(
            "start_delay",
            default_value="1.0",
            description="Seconds to wait after the first joint state (s)."),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="Build and print the goal without sending it."),

        hardware,
        trajectory_controller,
    ])
