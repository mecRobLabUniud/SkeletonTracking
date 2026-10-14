cmake_minimum_required(VERSION 3.10)
project(skeleton_tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

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

# ── Main ─────────────────────────────────────────────────────────────────────
add_executable(main 
    src/main.cpp
    src/trajectory_utils.cpp
    src/robot_model.cpp
	src/core.cpp
)
target_link_libraries(main PRIVATE 
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

# ── Temp ─────────────────────────────────────────────────────────────────────
add_executable(temp 
    src/temp.cpp
    src/trajectory_utils.cpp
    src/robot_model.cpp
	src/core.cpp
)
target_link_libraries(temp PRIVATE 
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