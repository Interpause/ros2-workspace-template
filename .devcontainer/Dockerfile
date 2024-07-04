FROM ros:humble-ros-base-jammy

RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Run each chunk of installs as separate to take advantage of layer cache.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    python3-pip \
    ~nros-${ROS_DISTRO}-rqt*

# Get latest mesa drivers for accelerated graphics.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    add-apt-repository ppa:oibaf/graphics-drivers -y
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    mesa-utils \
    vainfo \
    mesa-va-drivers

# Install version of Gazebo with WSL fix. Metapackage used here can be split further.
ARG GAZEBO_VERSION="garden"
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    gz-${GAZEBO_VERSION} \
    ros-${ROS_DISTRO}-ros-gz${GAZEBO_VERSION}

# Enable openCL support (OpenCV uses it for hardware acceleration).
RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Symlink python to python3 for convenience.
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Comment out below on Linux.
ENV LIBVA_DRIVER_NAME=d3d12
# Harmless on Linux, necessary on WSL for gui accel (compute accel works without).
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH}
