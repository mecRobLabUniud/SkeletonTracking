cmake_minimum_required(VERSION 3.10)
project(skeleton_tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# NOTE: do not add -march=native here. Pinocchio is installed as a prebuilt
# shared library (built without it) and PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
# makes this code use the library's explicit instantiations. A mismatch in
# CPU-feature/Eigen alignment between our objects and the library corrupts the
# heap (SIGSEGV in free / boost::bad_get during URDF loading).
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# ─────────────────────────────────────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(OpenMP REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Includes
# ─────────────────────────────────────────────────────────────────────────────
include_directories(
    ${ZMQ_INCLUDE_DIRS}
    include
)

# ─────────────────────────────────────────────────────────────────────────────
# Executables
# ─────────────────────────────────────────────────────────────────────────────

# ── RULA evaluation ──────────────────────────────────────────────────────────
add_executable(rula_evaluation
    src/rula_evaluation.cpp
    src/rula_score_computation.cpp
)

target_link_libraries(rula_evaluation PRIVATE
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    nlohmann_json::nlohmann_json
    pthread
    rt
)

# ── Main SSM ─────────────────────────────────────────────────────────────────
add_executable(main_SSM
    src/main_SSM.cpp
    src/trajectory_loader.cpp
    src/trajectory_utils.cpp
    src/calc_stop_duration.cpp
    src/SSM_core.cpp
    src/geometry.cpp
    src/min_distance_calculation.cpp
    src/skeleton_reader.cpp
    src/robot_model.cpp
    src/min_distance_calculation.cpp
)

target_link_options(main_SSM PRIVATE
    -Wl,--disable-new-dtags
    -Wl,-rpath,/usr/local/lib)

target_link_libraries(main_SSM PRIVATE
    stdc++fs
    pinocchio::pinocchio
    Eigen3::Eigen
    casadi
    nlohmann_json::nlohmann_json
    Threads::Threads
    ${ZMQ_LIBRARIES}
    rt   # shm_open / shm_unlink used by data_transmitter.hpp
    qpOASES
    /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
)

target_include_directories(main_SSM PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${ZMQ_INCLUDE_DIRS}
)

# ── Real-robot Waypoint bridge ───────────────────────────────────────────────
# Receives the WAYPOINT_15 stream published by main_SSM and streams it to the
# Panda through libfranka.
#
# libfranka must match the robot system version:
#   robot system 4.1.2  (FCI server version 4) -> /opt/libfranka-0.8.0
#   robot system >= 4.2.1 (server version 5)   -> /opt/libfranka
# Override with -DFRANKA_CMAKE_DIR=<prefix>/lib/cmake/Franka.
set(FRANKA_CMAKE_DIR "" CACHE PATH
    "libfranka CMake config directory (e.g. /opt/libfranka-0.8.0/lib/cmake/Franka)")
if(NOT FRANKA_CMAKE_DIR AND EXISTS "/opt/libfranka-0.8.0/lib/cmake/Franka")
    set(FRANKA_CMAKE_DIR "/opt/libfranka-0.8.0/lib/cmake/Franka")
endif()

if(FRANKA_CMAKE_DIR)
    find_package(Franka REQUIRED PATHS "${FRANKA_CMAKE_DIR}" NO_DEFAULT_PATH)
else()
    find_package(Franka QUIET)
endif()

if(Franka_FOUND)
    add_executable(robot_waypoint_bridge src/robot_waypoint_bridge.cpp)
    target_link_libraries(robot_waypoint_bridge PRIVATE
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        Threads::Threads
        ${ZMQ_LIBRARIES}
        Franka::Franka
        rt
    )
    target_include_directories(robot_waypoint_bridge PRIVATE ${ZMQ_INCLUDE_DIRS})
    message(STATUS "robot_waypoint_bridge: using libfranka from ${FRANKA_CMAKE_DIR}")
else()
    message(WARNING
        "libfranka not found; robot_waypoint_bridge will not be built. "
        "Pass -DFRANKA_CMAKE_DIR=<libfranka prefix>/lib/cmake/Franka.")
endif()

# ── Main SSMPFL ──────────────────────────────────────────────────────────────
add_executable(main_SSMPFL 
    src/main_SSMPFL.cpp
    src/trajectory_utils.cpp
    src/trajectory_loader.cpp
    src/robot_model.cpp
	src/SSMPFL_core.cpp
    src/geometry.cpp
    src/min_distance_calculation.cpp
)
target_link_libraries(main_SSMPFL PRIVATE 
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    nlohmann_json::nlohmann_json
    pthread
    rt
    qpOASES
    pinocchio::pinocchio
    OpenMP::OpenMP_CXX
)

# ── Main SSMPFLescape ────────────────────────────────────────────────────────
add_executable(main_SSMPFLescape 
    src/main_SSMPFLescape.cpp
    src/trajectory_utils.cpp
    src/trajectory_loader.cpp
    src/robot_model.cpp
	src/SSMPFLescape_core.cpp
    src/geometry.cpp
    src/min_distance_calculation.cpp
)
target_link_libraries(main_SSMPFLescape PRIVATE 
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    nlohmann_json::nlohmann_json
    pthread
    rt
    qpOASES
    pinocchio::pinocchio
    OpenMP::OpenMP_CXX
)


# ── Robot controller (ROS2 / multipanda_ros2) ─────────────────────────────
# ROS2 port of robot_controller.cpp.  It receives the ROBOT_12 stream and sends
# the accumulated trajectory through a FollowJointTrajectory action, like
# multipanda_ws/src/panda_test1_effort/src/test1_trajectory_sender.cpp.
# Built only when a ROS2 environment is sourced (rclcpp is found).
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
    find_package(rclcpp_action REQUIRED)
    find_package(control_msgs REQUIRED)
    find_package(trajectory_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(builtin_interfaces REQUIRED)

    add_executable(robot_controller_ros2
        src/robot_controller_ros2.cpp
        src/trajectory_utils.cpp
        src/robot_model.cpp
        src/trajectory_loader.cpp
    )
    target_link_libraries(robot_controller_ros2 PRIVATE
        Eigen3::Eigen
        pinocchio::pinocchio
        ${ZMQ_LIBRARIES}
        Threads::Threads
        nlohmann_json::nlohmann_json
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
        ${builtin_interfaces_TARGETS}
        ${control_msgs_TARGETS}
        ${trajectory_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
    )
    message(STATUS "robot_controller_ros2: building against the sourced ROS2 environment")
else()
    message(STATUS "rclcpp not found; robot_controller_ros2 skipped (source ROS2 to build it)")
endif()
