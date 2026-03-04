# robosense_ac_depth
[English](README.md) | [中文](README_CN.md)

## 1. Introduction

`robosense_ac_depth` is a stereo depth estimation project that includes:
- A TensorRT-based inference library for high-performance stereo depth computation. The network model comes from the state-of-the-art stereo depth estimation project [FoundationStereo](https://github.com/NVlabs/FoundationStereo). A platform with performance >= RTX3060 is recommended.
- ROS1/ROS2 wrappers for image subscription, disparity/depth/point cloud publishing, and visualization.
- Docker-based runtime environments for quick deployment and reproducible integration.

## 2. Code Structure

```
├── robosense_ac_depth/
│   └── dockerfiles             // Dockerfile
│       ├── ros1
│       ├── ros2
│   └── inference               // Depth estimation inference source code
│       ├── 3rdparty
│       ├── config
│       ├── include
│       ├── src
│   └── robosense_msgs          // Custom msgs
│   └── ros_wrapper             // ROS node project source code
│       ├── include
│       ├── launch
│       ├── rviz_config
│       ├── src
```

## 3. Docker Environment

### 3.1 Build Images
The project provides Dockerfiles for ROS1 and ROS2:
`dockerfiles/ros1/Dockerfile`
`dockerfiles/ros2/Dockerfile`

Run in the project root directory (ROS1):
```
docker build -t ubuntu20.04_ros1_cuda12.6_robosense_ac_depth:v1.0 -f dockerfiles/ros1/Dockerfile .
```

Run in the project root directory (ROS2):
```
docker build -t ubuntu22.04_ros2_cuda12.6_robosense_ac_depth:v1.0 -f dockerfiles/ros2/Dockerfile .
```

### 3.2 Start Containers

First, allow X11 on the host:
```
xhost +local:docker
```

#### 3.2.1 ROS1

Start ROS1 container:
```
docker run --rm -itd \
  --name robosense_ac_depth \
  --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ubuntu20.04_ros1_cuda12.6_robosense_ac_depth:v1.0 /bin/bash
```

Enter the ROS1 container and open a bash terminal:
```
docker exec -it robosense_ac_depth /bin/bash
```

Initialize ROS1 environment inside the container:
```
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
```

#### 3.2.2 ROS2

Start ROS2 container:
```
docker run --rm -itd \
  --name robosense_ac_depth \
  --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ubuntu22.04_ros2_cuda12.6_robosense_ac_depth:v1.0 /bin/bash
```

Enter the ROS2 container and open a bash terminal:
```
docker exec -it robosense_ac_depth /bin/bash
```

Initialize ROS2 environment inside the container:
```
source /opt/ros/humble/setup.bash
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
```

## 4. TensorRT Model Preparation

[Download](https://cdn.robosense.cn/models/stereo_model_vL.1.0.zip) the model file, unzip it, and copy it to the Docker container. Run on the host:
```
docker cp /path/to/models/stereo_model_vL.1.0.onnx robosense_ac_depth:/workspace/models/
```

Convert ONNX model to TensorRT model inside the container:
```
cd /workspace/models/
trtexec --onnx=stereo_model_vL.1.0.onnx --verbose --saveEngine=stereo_model_vL.1.0.plan --fp16
```

After successful conversion, the `stereo_model_vL.1.0.plan` model file will be generated.

## 5. Inference Library Compilation

Run the following commands in `/workspace` inside the container:

```
cd ./inference
mkdir build
cd build
cmake ..
make -j12
make install .
```

After compilation and installation, the inference library is installed by default in: `./ros_wrapper/lib_depth_inference`.

## 6. ROS1 Runtime

Run the following commands in `/workspace` inside the container:
```
mkdir catkin_ws
cd catkin_ws
mkdir src && cd src
ln -s ../../robosense_msgs ./
ln -s ../../roswrapper ./
cd ..
catkin_make
source devel/setup.bash
roslaunch robosense_ac_depth run_ros_node.launch

```

Play the bag file in the container or on the host:

```
rosbag play /path/to/bag_file.bag -l
```

Run RViz in the container or on the host:

```
rviz -d ros_wrapper/rviz_config/show_results.rviz
```

## 7. ROS2 Runtime

Run the following commands in `/workspace` inside the container:
```
mkdir colcon_ws
cd colcon_ws
mkdir src && cd src
ln -s ../../robosense_msgs ./
ln -s ../../roswrapper ./
cd ..
colcon build
source install/setup.bash
ros2 launch robosense_ac_depth run_ros2_node.launch.py

```

Play the ROS2 bag file in the container:

```
source /workspace/colcon_ws/install/setup.bash
ros2 bag play /path/to/bag_file -l
```

Run RViz inside the container:

```
rviz2 -d ros_wrapper/rviz_config/show_results_ros2.rviz
```

## 8. ROS Input/Output Messages

The ROS node topic names are configurable through launch parameters. Default names are:

Input topics:
- `left_image_topic`: `/rs_camera/left/color/image_raw` (`sensor_msgs/Image`)
- `right_image_topic`: `/rs_camera/right/color/image_raw` (`sensor_msgs/Image`)
- `device_calib_info_topic`: `/device_calib_info` (`robosense_msgs/RsACDeviceCalib`, used to receive device camera calibration parameters when `calib_file` is empty)

Output topics:
- `disparity_topic`: `/stereo/disparity` (`sensor_msgs/Image`)
- `rectified_left_topic`: `/stereo/left/rectified` (`sensor_msgs/Image`)
- `rectified_right_topic`: `/stereo/right/rectified` (`sensor_msgs/Image`)
- `pointcloud_topic`: `/stereo/dense/pointcloud` (`sensor_msgs/PointCloud2`)

## 9. Acknowledgements

We would like to thank related open-source projects and researchers. Special thanks to the authors of [EasyDeploy](https://github.com/codefuse-ai/EasyDeploy) and [FoundationStereo](https://github.com/NVlabs/FoundationStereo) for their excellent achievements and source code.

## 10. License

- **Code**: The source code of this project is licensed under the [Apache License 2.0](LICENSE).
- **Models**: The model files used with this project are licensed under the [NVIDIA Open Model License](https://www.nvidia.com/en-us/agreements/enterprise-software/nvidia-open-model-license/).
  - **Attribution**: Licensed by NVIDIA Corporation under the NVIDIA Open Model License.

> **Note**: Downloading, installing, or using model files recommended by this project means you agree to all terms of the NVIDIA Open Model License, including but not limited to commercial usage authorization and safety guidelines.
