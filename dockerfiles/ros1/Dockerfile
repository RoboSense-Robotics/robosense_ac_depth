FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    dirmngr \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS1 (Noetic) repository
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list

# Install ROS1 and common build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-opencv \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstools \
    ros-noetic-desktop-full \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    libyaml-cpp-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# TensorRT (expecting CUDA 12.6 repo in base image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    tensorrt \
    libnvinfer-dev \
    libnvinfer-plugin-dev \
    libnvonnxparsers-dev \
    libnvinfer-bin \
    && rm -rf /var/lib/apt/lists/* \
    && dpkg-query -W -f='${Version}\n' libnvinfer10 | grep -E '^10\.13\.'

# Ensure trtexec is directly callable from PATH
RUN set -eux; \
    if [ -x /usr/src/tensorrt/bin/trtexec ]; then \
      ln -sf /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec; \
    elif [ -x /usr/bin/trtexec ]; then \
      true; \
    else \
      echo "trtexec binary not found after TensorRT install"; \
      exit 1; \
    fi; \
    command -v trtexec

RUN rosdep init || true

RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace

COPY inference /workspace/inference

COPY ros_wrapper /workspace/ros_wrapper

COPY robosense_msgs /workspace/robosense_msgs

RUN mkdir /workspace/models

CMD ["/bin/bash"]
