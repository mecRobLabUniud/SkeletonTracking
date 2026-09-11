cmake_minimum_required(VERSION 3.10)
project(skeleton_tracking CXX)

# Richiesto C++14 per libfranka e Pinocchio
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─────────────────────────────────────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Franka 0.8.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# CasADi e Pinocchio
# find_package(casadi REQUIRED)
# find_package(pinocchio REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Includes
# ─────────────────────────────────────────────────────────────────────────────
include_directories(
    ${ZMQ_INCLUDE_DIRS}
    # ${CMAKE_SOURCE_DIR}
    # ${EIGEN3_INCLUDE_DIR}
    include
)

# ─────────────────────────────────────────────────────────────────────────────
# Executables
# ─────────────────────────────────────────────────────────────────────────────
# add_executable(robot_control 
#     src/robot_control.cpp 
#     src/examples_common.cpp
# )
# 
# target_link_libraries(robot_control PRIVATE 
#     Franka::Franka 
#     Eigen3::Eigen 
#     ${ZMQ_LIBRARIES} 
#     Threads::Threads
# )
# 
# add_executable(joint_point_to_point_motion 
#     src/joint_point_to_point_motion.cpp 
#     src/examples_common.cpp
# )
# target_link_libraries(joint_point_to_point_motion PRIVATE 
#     Franka::Franka 
#     Eigen3::Eigen
# )
# 
# add_executable(record_trajectory 
#     src/record_trajectory.cpp
# )
# target_link_libraries(record_trajectory PRIVATE 
#     Franka::Franka 
#     Eigen3::Eigen 
#     Threads::Threads
# )
# 
# add_executable(replay_trajectory 
#     src/replay_trajectory.cpp 
#     src/examples_common.cpp
# )
# target_link_libraries(replay_trajectory PRIVATE 
#     Franka::Franka 
#     Eigen3::Eigen
# )

# ── RULA evaluation ──────────────────────────────────────────────────────────
add_executable(rula_evaluation
    src/rula_evaluation.cpp
    src/rula_score_computation.cpp
)

target_link_libraries(rula_evaluation PRIVATE
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
    pthread
    rt
)

# ── Main ─────────────────────────────────────────────────────────────────────
add_executable(main 
    src/main.cpp
    src/trajectory_utils.cpp
    src/robot_model.cpp
    src/min_distance_calculation.cpp
	src/SSMPFL.cpp
)
target_link_libraries(main PRIVATE 
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
    pthread
    rt
    qpOASES
    pinocchio::pinocchio
)

# # ── Appendix A ───────────────────────────────────────────────────────────────
# add_executable(SSM_PFL_escape
#     src/SSM_PFL_escape.cpp
# )
# target_link_libraries(SSM_PFL_escape PRIVATE 
#     Eigen3::Eigen
# )
