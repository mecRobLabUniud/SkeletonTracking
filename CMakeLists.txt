cmake_minimum_required(VERSION 3.10)
project(skeleton_tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─────────────────────────────────────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

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