# SkeletonTracking + multipanda_ros2 development/runtime image.
#
# Base: Ubuntu 22.04 (Jammy) with ROS 2 Humble, because multipanda_ros2 only
# supports Humble.  The SkeletonTracking C++ side still needs a few libraries
# that are not packaged for Jammy (CasADi with IPOPT, Pinocchio with URDF +
# CasADi support, cppzmq >= 4.8 and qpOASES), so they are built from source
# below and installed into /usr/local.
#
# multipanda_ros2 additionally needs Eigen 3.3.9, MuJoCo 3.2.0 and libfranka
# 0.9.2, which are installed into /usr/local, /opt/mujoco and /opt/libfranka.

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV LD_LIBRARY_PATH=/usr/local/lib:/opt/mujoco/lib:/opt/libfranka/lib
ENV CMAKE_PREFIX_PATH=/usr/local:/opt/mujoco:/opt/libfranka

# ── APT sources ──────────────────────────────────────────────────────────────
# Use the USTC mirrors (the default Ubuntu / ROS servers are unreliable from
# this network).  The ros:humble base image does not ship a ROS apt source, so
# add one for the extra ros-humble-* packages.
RUN printf '%s\n' \
      'deb http://mirrors.ustc.edu.cn/ubuntu/ jammy main restricted universe multiverse' \
      'deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse' \
      'deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse' \
      'deb http://mirrors.ustc.edu.cn/ubuntu/ jammy-security main restricted universe multiverse' \
      > /etc/apt/sources.list && \
    printf 'Acquire::Retries "5";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' \
      > /etc/apt/apt.conf.d/99retries && \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list && \
    rm -rf /var/lib/apt/lists/*

# ── Build tools + SkeletonTracking dependencies ──────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gfortran \
    cmake \
    ninja-build \
    ccache \
    pkg-config \
    git \
    curl \
    wget \
    python3 \
    python3-dev \
    python3-pip \
    libboost-all-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    nlohmann-json3-dev \
    libzmq3-dev \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
    libpoco-dev \
    libfmt-dev \
    libyaml-cpp-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxi-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# ── multipanda_ros2 / ROS 2 dependencies ─────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-control-msgs \
    ros-humble-angles \
    ros-humble-generate-parameter-library \
    ros-humble-backward-ros \
    ros-humble-moveit \
    ros-humble-warehouse-ros-sqlite \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-nav-msgs \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rosgraph-msgs \
    ros-humble-lifecycle-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-urdf \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ros2param \
    ros-humble-ros2run \
    ros-humble-rcpputils \
    ros-humble-ament-index-cpp \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-clang-format \
    ros-humble-ament-flake8 \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-launch-testing \
    ros-humble-rosbridge-server \
    ros-humble-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-controller-manager \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-plot \
    && rm -rf /var/lib/apt/lists/*

# Runtime dependencies for OpenCV / MediaPipe / RealSense / the GUI stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libice6 \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxfixes3 \
    libusb-1.0-0 \
    libegl1 \
    libgles2 \
    libgl1-mesa-dri \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Eigen 3.3.9 (multipanda_ros2/libfranka need 3.3.x) ───────────────────────
# Installed into /usr/local so it takes precedence over Jammy's Eigen 3.4 in
# /usr/include/eigen3 (CMake always searches /usr/local first).
RUN cd /tmp && \
    curl -fsSL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o eigen.tar.gz && \
    tar -xzf eigen.tar.gz && \
    cmake -S eigen-3.3.9 -B eigen-3.3.9/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --install eigen-3.3.9/build && \
    ldconfig && \
    rm -rf /tmp/eigen*

# ── cppzmq (C++ binding with the zmq::sockopt API used by the project) ────────
RUN cd /tmp && \
    git clone --depth 1 --branch v4.8.1 https://github.com/zeromq/cppzmq.git && \
    cmake -S cppzmq -B cppzmq/build \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCPPZMQ_BUILD_TESTS=OFF && \
    cmake --build cppzmq/build -j"$(nproc)" && \
    cmake --install cppzmq/build && \
    ldconfig && \
    rm -rf /tmp/cppzmq

# ── CasADi with the system IPOPT plugin ──────────────────────────────────────
# calc_stop_duration.cpp uses casadi::nlpsol("...", "ipopt", ...).  A minimal
# build is enough: only the IPOPT and LAPACK interfaces are required.
RUN cd /tmp && \
    git clone --depth 1 --branch 3.6.7 https://github.com/casadi/casadi.git && \
    cmake -S casadi -B casadi/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_PREFIX_PATH=/usr/local \
      -DWITH_PYTHON=OFF \
      -DWITH_MATLAB=OFF \
      -DWITH_OCTAVE=OFF \
      -DWITH_JSON=OFF \
      -DWITH_EXAMPLES=OFF \
      -DWITH_DOC=OFF \
      -DWITH_IPOPT=ON \
      -DWITH_BUILD_IPOPT=OFF \
      -DWITH_LAPACK=ON \
      -DWITH_QPOASES=OFF \
      -DWITH_BLOCKSQP=OFF \
      -DWITH_SUPERSCS=OFF \
      -DWITH_OSQP=OFF \
      -DWITH_PROXQP=OFF \
      -DWITH_HIGHS=OFF \
      -DWITH_DAQP=OFF \
      -DWITH_SLEQP=OFF \
      -DWITH_BONMIN=OFF \
      -DWITH_CBC=OFF \
      -DWITH_CLP=OFF \
      -DWITH_MUMPS=OFF \
      -DWITH_SPRAL=OFF \
      -DWITH_HSL=OFF \
      -DWITH_AMPL=OFF \
      -DWITH_SLICOT=OFF \
      -DWITH_OOQP=OFF \
      -DWITH_SQIC=OFF \
      -DWITH_KNITRO=OFF \
      -DWITH_SNOPT=OFF \
      -DWITH_WORHP=OFF \
      -DWITH_CPLEX=OFF \
      -DWITH_GUROBI=OFF \
      -DWITH_MADNLP=OFF \
      -DWITH_BLASFEO=OFF \
      -DWITH_HPIPM=OFF \
      -DWITH_FATROP=OFF \
      -DWITH_SUNDIALS=OFF \
      -DWITH_BUILD_SUNDIALS=OFF \
      -DWITH_CSPARSE=OFF \
      -DWITH_BUILD_CSPARSE=OFF \
      -DWITH_TINYXML=OFF \
      -DWITH_BUILD_TINYXML=OFF \
      -DWITH_FMI2=OFF \
      -DWITH_OPENCL=OFF \
      -DWITH_CLANG=OFF \
      -DWITH_DEEPBIND=OFF \
      -DWITH_OPENMP=OFF \
      -DWITH_MOCKUP_REQUIRED=OFF \
      -DWITH_BUILD_REQUIRED=OFF && \
    cmake --build casadi/build -j"$(nproc)" && \
    cmake --install casadi/build && \
    ldconfig && \
    rm -rf /tmp/casadi

# ── Pinocchio (URDF parser + CasADi support) ─────────────────────────────────
RUN cd /tmp && \
    git clone --recursive --branch v2.7.0 https://github.com/stack-of-tasks/pinocchio.git && \
    cmake -S pinocchio -B pinocchio/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_PREFIX_PATH=/usr/local \
      -DBUILD_PYTHON_INTERFACE=OFF \
      -DBUILD_TESTING=OFF \
      -DBUILD_BENCHMARK=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_WITH_URDF_SUPPORT=ON \
      -DBUILD_WITH_COLLISION_SUPPORT=OFF \
      -DBUILD_WITH_CASADI_SUPPORT=ON && \
    cmake --build pinocchio/build -j"$(nproc)" && \
    cmake --install pinocchio/build && \
    ldconfig && \
    rm -rf /tmp/pinocchio

# ── qpOASES ──────────────────────────────────────────────────────────────────
RUN cd /tmp && \
    git clone --branch releases/3.2.1 --depth 1 https://github.com/coin-or/qpOASES.git && \
    cmake -S qpOASES -B qpOASES/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DQPOASES_BUILD_EXAMPLES=OFF && \
    cmake --build qpOASES/build -j"$(nproc)" && \
    cmake --install qpOASES/build && \
    ldconfig && \
    rm -rf /tmp/qpOASES

# ── MuJoCo 3.2.0 (needed by mujoco_ros_pkgs) ────────────────────────────────
RUN cd /tmp && \
    git clone --depth 1 --branch 3.2.0 https://github.com/google-deepmind/mujoco.git && \
    cmake -S mujoco -B mujoco/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/mujoco \
      -DMUJOCO_BUILD_EXAMPLES=OFF \
      -DMUJOCO_BUILD_TESTS=OFF && \
    cmake --build mujoco/build -j"$(nproc)" && \
    cmake --install mujoco/build && \
    ldconfig && \
    rm -rf /tmp/mujoco

# ── libfranka 0.9.2 (needed by franka_hardware) ─────────────────────────────
RUN cd /tmp && \
    ( for i in 1 2 3; do \
        rm -rf libfranka; \
        GIT_TERMINAL_PROMPT=0 git clone --recursive https://github.com/frankaemika/libfranka.git && exit 0; \
        sleep 5; \
      done; exit 1 ) && \
    cd libfranka && \
    git checkout 0.9.2 && \
    git submodule update --init --recursive && \
    cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/libfranka \
      -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    cd /tmp && rm -rf /tmp/libfranka && ldconfig

# ── DQ Robotics (needed by panda_motion_generators) ─────────────────────────
RUN apt-get update && \
    add-apt-repository -y ppa:dqrobotics-dev/release && \
    apt-get update && \
    apt-get install -y --no-install-recommends libdqrobotics && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/*

# ── Python dependencies ──────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --upgrade pip
COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir --ignore-installed 'blinker>=1.6.2' && \
    python3 -m pip install --no-cache-dir -r /tmp/requirements.txt && \
    rm -f /tmp/requirements.txt

WORKDIR /home/lab/SkeletonTracking

# ── SkeletonTracking source and C++ build ────────────────────────────────────
COPY . .

RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/usr/local && \
    cmake --build build -j"$(nproc)"

# ── multipanda_ros2 source and ROS 2 build ───────────────────────────────────
# Pinned to the revision that was tested; the submodule points at the required
# mujoco_ros_pkgs fork/branch.
ARG MULTIPANDA_COMMIT=44076f19b52706259d9b22a48341e729f7e36ee1
RUN mkdir -p /home/lab/multipanda_ws/src && \
    ( for i in 1 2 3; do \
        rm -rf /home/lab/multipanda_ws/src/multipanda_ros2; \
        GIT_TERMINAL_PROMPT=0 git clone --recursive https://github.com/tenfoldpaper/multipanda_ros2.git \
          /home/lab/multipanda_ws/src/multipanda_ros2 && exit 0; \
        sleep 5; \
      done; exit 1 ) && \
    git -C /home/lab/multipanda_ws/src/multipanda_ros2 checkout "${MULTIPANDA_COMMIT}" && \
    git -C /home/lab/multipanda_ws/src/multipanda_ros2 submodule update --init --recursive

RUN . /opt/ros/humble/setup.sh && \
    cd /home/lab/multipanda_ws && \
    colcon build \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF

# Make ROS and the built workspace available in plain `bash` sessions too.
RUN { \
      echo 'source /opt/ros/humble/setup.bash'; \
      echo 'if [ -f /home/lab/multipanda_ws/install/setup.bash ]; then source /home/lab/multipanda_ws/install/setup.bash; fi'; \
    } >> /root/.bashrc && \
    { \
      echo 'source /opt/ros/humble/setup.bash'; \
      echo 'if [ -f /home/lab/multipanda_ws/install/setup.bash ]; then source /home/lab/multipanda_ws/install/setup.bash; fi'; \
    } >> /etc/bash.bashrc

EXPOSE 5000

# Keep the container alive as an interactive development shell.  Start a run
# with e.g.:
#   docker compose run --rm app ./run.sh --stream --test 7 --robot --traj 3 --gui
# or work on the ROS 2 workspace:
#   cd /home/lab/multipanda_ws && colcon build
CMD ["/bin/bash"]
