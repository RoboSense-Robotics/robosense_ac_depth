from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument("config_file", default_value="/workspace/ros_wrapper/lib_depth_inference/config/config.yaml")
    
    calib_file_arg = DeclareLaunchArgument("calib_file", default_value="")

    left_image_topic_arg = DeclareLaunchArgument(
        "left_image_topic", default_value="/rs_camera/left/color/image_raw"
    )
    right_image_topic_arg = DeclareLaunchArgument(
        "right_image_topic", default_value="/rs_camera/right/color/image_raw"
    )
    crop_img_arg = DeclareLaunchArgument("crop_img", default_value="[0, 0, 0, 0]")

    disparity_topic_arg = DeclareLaunchArgument("disparity_topic", default_value="/stereo/disparity")
    rectified_left_topic_arg = DeclareLaunchArgument(
        "rectified_left_topic", default_value="/stereo/left/rectified"
    )
    rectified_right_topic_arg = DeclareLaunchArgument(
        "rectified_right_topic", default_value="/stereo/right/rectified"
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/stereo/dense/pointcloud"
    )

    publish_rectified_arg = DeclareLaunchArgument("publish_rectified", default_value="true")
    save_results_arg = DeclareLaunchArgument("save_results", default_value="false")
    save_results_path_arg = DeclareLaunchArgument("save_results_path", default_value="")

    queue_size_arg = DeclareLaunchArgument("queue_size", default_value="10")
    time_sync_threshold_arg = DeclareLaunchArgument("time_sync_threshold", default_value="0.01")

    ros_node = Node(
        package="robosense_ac_depth",
        executable="ros_node",
        name="ros_node",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "calib_file": LaunchConfiguration("calib_file"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "disparity_topic": LaunchConfiguration("disparity_topic"),
                "rectified_left_topic": LaunchConfiguration("rectified_left_topic"),
                "rectified_right_topic": LaunchConfiguration("rectified_right_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "publish_rectified": LaunchConfiguration("publish_rectified"),
                "queue_size": LaunchConfiguration("queue_size"),
                "time_sync_threshold": LaunchConfiguration("time_sync_threshold"),
                "crop_img": LaunchConfiguration("crop_img"),
                "save_results": LaunchConfiguration("save_results"),
                "save_results_path": LaunchConfiguration("save_results_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            calib_file_arg,
            left_image_topic_arg,
            right_image_topic_arg,
            crop_img_arg,
            disparity_topic_arg,
            rectified_left_topic_arg,
            rectified_right_topic_arg,
            pointcloud_topic_arg,
            publish_rectified_arg,
            save_results_arg,
            save_results_path_arg,
            queue_size_arg,
            time_sync_threshold_arg,
            ros_node,
        ]
    )
