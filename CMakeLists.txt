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

# ── Main SSMPFL ──────────────────────────────────────────────────────────────
add_executable(main_SSMPFL 
    src/main_SSMPFL.cpp
    src/trajectory_utils.cpp
    src/trajectory_loader.cpp
    src/robot_model.cpp
	src/core.cpp
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
	src/core.cpp
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

# ── Main SSM ─────────────────────────────────────────────────────────────────
add_executable(main_SSM
    src/main_SSM.cpp
    src/trajectory_loader.cpp
    src/ik_solver.cpp
    src/calc_stop_duration.cpp
    src/flag_stop.cpp
    src/geometry.cpp
    src/skeleton_reader.cpp
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
    /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
    /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
)

target_include_directories(main_SSM PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${ZMQ_INCLUDE_DIRS}
)