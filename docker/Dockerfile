# SkeletonTracking development/runtime image.
#
# The C++ side needs a few libraries that are not packaged for Ubuntu 20.04
# (CasADi with IPOPT, Pinocchio with URDF + CasADi support, cppzmq >= 4.8 and
# qpOASES), so they are built from source below.  Everything is installed into
# /usr/local, which is on the default compiler/linker search path.

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV LD_LIBRARY_PATH=/usr/local/lib

# Use the USTC mirror (archive.ubuntu.com is unreliable from this network)
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list && \
    printf 'Acquire::Retries "5";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' > /etc/apt/apt.conf.d/99retries

# ── System dependencies ──────────────────────────────────────────────────────
# build tools + C++ dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gfortran \
    cmake \
    pkg-config \
    git \
    curl \
    wget \
    ca-certificates \
    python3 \
    python3-dev \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    nlohmann-json3-dev \
    libzmq3-dev \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
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
    usbutils \
    && rm -rf /var/lib/apt/lists/*

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

# ── Python dependencies ──────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --upgrade pip
COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt && \
    rm -f /tmp/requirements.txt

WORKDIR /home/lab/SkeletonTracking

# ── Project source and C++ build ─────────────────────────────────────────────
COPY . .

RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/usr/local && \
    cmake --build build -j"$(nproc)"

EXPOSE 5000

# Keep the container alive as an interactive development shell.  Start a run
# with e.g.:
#   docker compose run --rm app ./run.sh --stream --test 7 --robot --traj 3 --gui
CMD ["/bin/bash"]
