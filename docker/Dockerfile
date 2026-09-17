# Use a base image with build tools
FROM ubuntu/nginx:1.18-20.04_edge

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Use the USTC mirror (archive.ubuntu.com is unreliable from this network)
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list && \
    printf 'Acquire::Retries "5";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' > /etc/apt/apt.conf.d/99retries

# Install system deps: build tools + Python
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    python3 \
    python3-pip \
    wget \
    libgl1 \
    libeigen3-dev \
    libopencv-dev \
    libzmq3-dev \
    nlohmann-json3-dev \
    libboost-all-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libpoco-dev \
    libglib2.0-0 \
    libusb-1.0-0 \
    libegl1 \
    libgles2 \
    libgl1-mesa-dri \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    wget https://github.com/zeromq/libzmq/releases/download/v4.3.5/zeromq-4.3.5.tar.gz && \
    tar -xzf zeromq-4.3.5.tar.gz && \
    cd zeromq-4.3.5 && \
    ./configure --prefix=/usr/local && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/zeromq-4.3.5*

# Install cppzmq (C++ binding with zmq::sockopt API used by the project)
RUN cd /tmp && \
    git clone --depth 1 --branch v4.8.1 https://github.com/zeromq/cppzmq.git && \
    cmake -S cppzmq -B cppzmq/build \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCPPZMQ_BUILD_TESTS=OFF && \
    cmake --build cppzmq/build && \
    cmake --install cppzmq/build && \
    rm -rf /tmp/cppzmq

# Install Pinocchio from source (not available in Ubuntu 20.04 repos)
RUN cd /tmp && \
    git clone --recursive --branch v2.7.0 https://github.com/stack-of-tasks/pinocchio.git && \
    cd pinocchio && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_PYTHON_INTERFACE=OFF \
      -DBUILD_TESTING=OFF \
      -DBUILD_BENCHMARK=OFF \
      -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/pinocchio

# Install libfranka (not available in Ubuntu 20.04 repos)
RUN cd /tmp && \
    git clone --recursive --branch 0.8.0 --depth 1 https://github.com/frankaemika/libfranka.git && \
    cd libfranka && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/libfranka

# Install qpOASES (not available in Ubuntu 20.04 repos)
RUN cd /tmp && \
    git clone --branch releases/3.2.1 --depth 1 https://github.com/coin-or/qpOASES.git && \
    cd qpOASES && \
    cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/qpOASES

RUN python3 -m pip install --upgrade pip

WORKDIR /home/lab

# Install Python dependencies first (better layer caching)
COPY docker/requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && \
    rm -f requirements.txt

# Copy the project source
COPY . SkeletonTracking/

WORKDIR /home/lab/SkeletonTracking

# Build the C++ portion
RUN mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_PREFIX_PATH=/usr/local && \
    make -j$(nproc)

# Default command — adjust to your entrypoint
# CMD ["python3", "src/main.py"]