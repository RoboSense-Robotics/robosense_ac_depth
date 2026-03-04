# robosense_ac_depth
[English](README.md) | [中文](README_CN.md)

## 1. 项目简介

`robosense_ac_depth` 是一个双目深度估计项目，主要包含：
- 基于 TensorRT 的高性能推理库，用于双目深度推理计算，网络模型来自先进的双目深度估计项目[FoundationStereo](https://github.com/NVlabs/FoundationStereo)，建议运行在性能≥RTX3060的平台上。
- ROS1/ROS2 封装节点，用于图像订阅、视差/深度/点云发布与可视化。
- 基于 Docker 的运行环境构建，便于快速部署和一致性集成。

## 2. 代码框架

```
├── robosense_ac_depth/
│   └── dockerfiles             // Dockerfile
│       ├── ros1
│       ├── ros2
│   └── inference               // 深度估计推理源码
│       ├── 3rdparty
│       ├── config
│       ├── include
│       ├── src
│   └── robosense_msgs          // 自定义msgs
│   └── ros_wrapper             // ros节点工程源码
│       ├── include
│       ├── launch
│       ├── rviz_config
│       ├── src
```
## 3. Docker环境

### 3.1 构建镜像
项目提供ROS1和ROS2的 Dockerfile：
`dockerfiles/ros1/Dockerfile`
`dockerfiles/ros2/Dockerfile`

在项目根目录执行（ros1）：
```
docker build -t ubuntu20.04_ros1_cuda12.6_robosense_ac_depth:v1.0 -f dockerfiles/ros1/Dockerfile .
```
在项目根目录执行（ros2）：
```
docker build -t ubuntu22.04_ros2_cuda12.6_robosense_ac_depth:v1.0 -f dockerfiles/ros2/Dockerfile .
```

### 3.2 启动容器

先在宿主机允许 X11：
```
xhost +local:docker
```
#### 3.2.1 ROS1

启动ROS1容器：
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

进入ROS1容器并打开bash终端
```
docker exec -it robosense_ac_depth /bin/bash
```

容器内初始化ROS1环境：
```
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
```

#### 3.2.2 ROS2

启动ROS2容器：
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

进入ROS2容器并打开bash终端
```
docker exec -it robosense_ac_depth /bin/bash
```

容器内初始化ROS2环境：
```
source /opt/ros/humble/setup.bash
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
```

## 4. TensorRT模型准备

[下载](https://cdn.robosense.cn/models/stereo_model_vL.1.0.zip)网络模型文件，将模型文件解压后拷贝到Docker容器中，宿主机中执行拷贝指令：
```
docker cp /path/to/models/stereo_model_vL.1.0.onnx robosense_ac_depth:/workspace/models/
```

在容器中将ONNX模型转换为TensorRT模型：
```
cd /workspace/models/
trtexec --onnx=stereo_model_vL.1.0.onnx --verbose --saveEngine=stereo_model_vL.1.0.plan --fp16
```
转换成功后将生成stereo_model_vL.1.0.plan的模型文件。

## 5. 算法库编译
在容器/workspace目录中执行以下指令：

```
cd ./inference
mkdir build
cd build
cmake ..
make -j12
make install .
```
编译安装完成后，推理库默认安装在：./ros_wrapper/lib_depth_inference目录下。

## 6. ROS1运行

在容器/workspace目录中执行以下指令：
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
容器或宿主机中播放bag包：

```
rosbag play /path/to/bag_file.bag -l
```

容器或宿主机中运行rviz

```
rviz -d ros_wrapper/rviz_config/show_results.rviz
```

## 7. ROS2运行

在容器/workspace目录中执行以下指令：
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

容器中播放ros2的bag包：

```
source /workspace/colcon_ws/install/setup.bash
ros2 bag play /path/to/bag_file -l
```

容器中运行rviz2

```
rviz2 -d ros_wrapper/rviz_config/show_results_ros2.rviz
```


## 8. ROS输入/输出消息

ROS节点的 topic 名称可通过 launch 参数配置，默认命名如下：

输入消息：
- `left_image_topic`：`/rs_camera/left/color/image_raw`（`sensor_msgs/Image`）
- `right_image_topic`：`/rs_camera/right/color/image_raw`（`sensor_msgs/Image`）
- `device_calib_info_topic`：`/device_calib_info`（`robosense_msgs/RsACDeviceCalib`，当 `calib_file` 为空时用于接收设备相机标定参数）

输出消息：
- `disparity_topic`：`/stereo/disparity`（`sensor_msgs/Image`）
- `rectified_left_topic`：`/stereo/left/rectified`（`sensor_msgs/Image`）
- `rectified_right_topic`：`/stereo/right/rectified`（`sensor_msgs/Image`）
- `pointcloud_topic`：`/stereo/dense/pointcloud`（`sensor_msgs/PointCloud2`）

## 9. 致谢

在此对相关开源项目及研究人员表示感谢，衷心的感谢[EasyDeploy](https://github.com/codefuse-ai/EasyDeploy)、[FoundationStereo](https://github.com/NVlabs/FoundationStereo)的作者提供的卓越成果和源码。

## 10. 许可

- **代码 (Code)**: 本项目源码遵循 [Apache License 2.0](LICENSE) 协议。
- **模型 (Models)**: 本项目配套使用的模型文件遵循 [NVIDIA Open Model License](https://www.nvidia.com/en-us/agreements/enterprise-software/nvidia-open-model-license/)。
  - **归属声明**: Licensed by NVIDIA Corporation under the NVIDIA Open Model License.

> **注意**：下载、安装或使用本项目推荐的模型文件，即代表您同意 NVIDIA Open Model License 的所有条款，包括但不限于商用授权及安全准则。
