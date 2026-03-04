FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04

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

# ROS2 (Humble) repository for Ubuntu 22.04
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS2 and common build tools/libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-opencv \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-tools \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    libyaml-cpp-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# TensorRT: pin to 10.13.x + cuda12.9 (avoid apt pulling latest 10.15/cuda13.1)
RUN set -eux; \
    apt-get update; \
    pick_ver() { \
      pkg="$1"; \
      apt-cache madison "${pkg}" | awk '/10\.13\..*\+cuda12\.9/{print $3; exit}'; \
    }; \
    TRT_DEV_VER="$(pick_ver libnvinfer-dev)"; \
    TRT_RT_VER="$(pick_ver libnvinfer10)"; \
    TRT_PLUGIN_DEV_VER="$(pick_ver libnvinfer-plugin-dev)"; \
    TRT_PLUGIN_RT_VER="$(pick_ver libnvinfer-plugin10)"; \
    TRT_ONNX_DEV_VER="$(pick_ver libnvonnxparsers-dev)"; \
    TRT_ONNX_RT_VER="$(pick_ver libnvonnxparsers10)"; \
    TRT_BIN_VER="$(pick_ver libnvinfer-bin)"; \
    TRT_LEAN_VER="$(pick_ver libnvinfer-lean10)"; \
    TRT_DISPATCH_VER="$(pick_ver libnvinfer-dispatch10)"; \
    TRT_VC_PLUGIN_VER="$(pick_ver libnvinfer-vc-plugin10)"; \
    TRT_HEADERS_DEV_VER="$(pick_ver libnvinfer-headers-dev)"; \
    TRT_HEADERS_PLUGIN_DEV_VER="$(pick_ver libnvinfer-headers-plugin-dev)"; \
    test -n "${TRT_DEV_VER}" && test -n "${TRT_RT_VER}" && test -n "${TRT_PLUGIN_DEV_VER}" && \
      test -n "${TRT_PLUGIN_RT_VER}" && test -n "${TRT_ONNX_DEV_VER}" && test -n "${TRT_ONNX_RT_VER}" && \
      test -n "${TRT_BIN_VER}" && test -n "${TRT_LEAN_VER}" && test -n "${TRT_DISPATCH_VER}" && \
      test -n "${TRT_VC_PLUGIN_VER}" && test -n "${TRT_HEADERS_DEV_VER}" && \
      test -n "${TRT_HEADERS_PLUGIN_DEV_VER}"; \
    apt-get install -y --no-install-recommends \
      "libnvinfer-dev=${TRT_DEV_VER}" \
      "libnvinfer10=${TRT_RT_VER}" \
      "libnvinfer-plugin-dev=${TRT_PLUGIN_DEV_VER}" \
      "libnvinfer-plugin10=${TRT_PLUGIN_RT_VER}" \
      "libnvonnxparsers-dev=${TRT_ONNX_DEV_VER}" \
      "libnvonnxparsers10=${TRT_ONNX_RT_VER}" \
      "libnvinfer-bin=${TRT_BIN_VER}" \
      "libnvinfer-lean10=${TRT_LEAN_VER}" \
      "libnvinfer-dispatch10=${TRT_DISPATCH_VER}" \
      "libnvinfer-vc-plugin10=${TRT_VC_PLUGIN_VER}" \
      "libnvinfer-headers-dev=${TRT_HEADERS_DEV_VER}" \
      "libnvinfer-headers-plugin-dev=${TRT_HEADERS_PLUGIN_DEV_VER}"; \
    rm -rf /var/lib/apt/lists/*; \
    dpkg-query -W -f='${Package} ${Version}\n' \
      libnvinfer10 libnvinfer-dev libnvinfer-bin \
      libnvinfer-lean10 libnvinfer-dispatch10 libnvinfer-vc-plugin10 \
      libnvinfer-headers-dev libnvinfer-headers-plugin-dev | tee /tmp/trt_versions.txt; \
    grep -E '10\.13\..*\+cuda12\.9' /tmp/trt_versions.txt

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

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace

COPY inference /workspace/inference

COPY ros_wrapper /workspace/ros_wrapper

COPY robosense_msgs /workspace/robosense_msgs

RUN mkdir -p /workspace/models

CMD ["/bin/bash"]
