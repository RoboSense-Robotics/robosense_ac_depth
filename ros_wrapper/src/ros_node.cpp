#if ROS_VERSION == 1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#define ROS_INFO_IMPL ROS_INFO
#define ROS_ERROR_IMPL ROS_ERROR
#define ROS_WARN_IMPL ROS_WARN
#define ROS_INFO_THROTTLE_IMPL ROS_INFO_THROTTLE
#include <XmlRpcValue.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <sstream>
#include <cctype>
#include <future>
#include <deque>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "depth_inference_api.h"
#include <robosense_msgs/RsACDeviceCalib.h>

class StereoNode {
public:
  StereoNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
    // 1. First initialize ROS parameters and create the inference object
    InitializeROS();
    
    // 2. Then validate the calibration file and initialize the inference engine
    ROS_INFO("Checking calibration file: %s", calib_file_.c_str());
    std::ifstream ifs_calib_file(calib_file_);

    if (!ifs_calib_file.good()) {
        ROS_ERROR("Sensor calibration file not found: %s", calib_file_.c_str());
        return;
    }
    ifs_calib_file.close();
    
    ROS_INFO("Initializing depth inference engine...");
    if (!depth_inference_->initialize(config_file_, calib_file_)) {
        ROS_ERROR("Depth inference initialization failed!");
        return;
    }
    depth_inference_params_ = depth_inference_->GetDepthInferenceParams();
    // 3. Create folders to save results
    if (save_results_) {      
      std::string model_name = depth_inference_params_.model_file.substr(depth_inference_params_.model_file.find_last_of("/") + 1);
      // Use C++ standard library to create the result directory with algorithm version
      save_results_path_ = save_results_path_ + "/" + depth_inference_params_.version_num + "/" + model_name;
      std::filesystem::create_directories(save_results_path_);
      ROS_INFO("Save results to %s",  save_results_path_.c_str());
      std::string disparity_path = save_results_path_ + "/disparity";
      std::filesystem::create_directories(disparity_path);
      ROS_INFO(" Save disparity results to %s",  disparity_path.c_str());
      std::string depth_path = save_results_path_ + "/depth";
      std::filesystem::create_directories(depth_path);
      ROS_INFO(" Save depth results to %s",  depth_path.c_str());
      std::string pcd_path = save_results_path_ + "/points";
      std::filesystem::create_directories(pcd_path);
      ROS_INFO(" Save pcd results to %s",  pcd_path.c_str());
      std::string left_path = save_results_path_ + "/left";
      std::filesystem::create_directories(left_path);
      ROS_INFO(" Save left results to %s",  left_path.c_str());
    }
    
    // 4. Set inference result callback (must be set before image reception starts)
    ROS_INFO("Setting inference callback...");
    depth_inference_->setResultsCallback(std::bind(&StereoNode::ResultHandler, this, std::placeholders::_1));
    depth_inference_->start();

    ROS_INFO("Stereo Node initialized successfully!");
    ROS_INFO("Async inference pipeline enabled.");
  }

  ~StereoNode() = default;

  void Run()
  {
    // Start node (callback has already been set in constructor)
    ROS_INFO("Stereo Node is ready to process images.");
  }

  void Stop()
  {
    // Stop node
    depth_inference_->stop();
    ROS_INFO("Stopping Stereo Node.");
  }

private:

  static bool ParseCropString(const std::string &value, std::vector<int> &out)
  {
    out.clear();
    std::string cleaned = value;
    for (char &ch : cleaned)
    {
      if (!(std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+'))
        ch = ' ';
    }
    std::stringstream ss(cleaned);
    int v = 0;
    while (ss >> v)
    {
      out.push_back(v);
    }
    return out.size() == 4;
  }

  void LoadCropParam()
  {
    crop_img_ = {0, 0, 0, 0};

    XmlRpc::XmlRpcValue crop_val;
    if (!nh_private_.getParam("crop_img", crop_val))
    {
      ROS_WARN("crop_img not set. Using default [0, 0, 0, 0].");
      return;
    }

    std::vector<int> parsed;
    if (crop_val.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < crop_val.size(); ++i)
      {
        if (crop_val[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          parsed.push_back(static_cast<int>(crop_val[i]));
        } else if (crop_val[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
          parsed.push_back(static_cast<int>(static_cast<double>(crop_val[i])));
        } else
        {
          ROS_WARN("crop_img[%d] has unsupported type. Using default.", i);
          parsed.clear();
          break;
        }
      }
    } else if (crop_val.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      ParseCropString(static_cast<std::string>(crop_val), parsed);
    }

    if (parsed.size() == 4)
    {
      crop_img_ = parsed;
    } else
    {
      ROS_WARN("Invalid crop_img, expected 4 ints. Using default [0, 0, 0, 0].");
      crop_img_ = {0, 0, 0, 0};
    }
  }

  void GetDeviceCalibrationInfo()
  {
    ros::Duration timeout(20);                
    std::string device_calib_info_topic = "/device_calib_info"; 
    ROS_INFO("Waiting 20sec for RsACDeviceCalib on topic: %s", device_calib_info_topic.c_str());
    auto device_calib_info = ros::topic::waitForMessage<robosense_msgs::RsACDeviceCalib>(device_calib_info_topic, nh_private_, timeout);
    if(!device_calib_info)
    {
      ROS_ERROR("Failed to receive device calibration info.");
      return;
    } else {
      ROS_INFO("Received RsACDeviceCalib.");
      std::string device_frame = device_calib_info->header.frame_id;
      YAML::Node calib_yaml;
      calib_yaml["DEVICE_ID"] = device_frame;
      // 保存Lidar外参
      calib_yaml["Sensor"]["Lidar"]["base_link"] = "sensor";
      calib_yaml["Sensor"]["Lidar"]["frame"] = "lidar";
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["x"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["y"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["z"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["x"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["y"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["z"] = 0.0;
      calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["w"] = 1.0;             
      
      //保存左相机参数
      // 外参
      calib_yaml["Sensor"]["Camera"]["base_link"] = "sensor";
      calib_yaml["Sensor"]["Camera"]["frame"] = "camera";
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["x"] = device_calib_info->camtolidartx;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["y"] = device_calib_info->camtolidarty;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["z"] = device_calib_info->camtolidartz;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["x"] = device_calib_info->camtolidarqx;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["y"] = device_calib_info->camtolidarqy;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["z"] = device_calib_info->camtolidarqz;
      calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["w"] = device_calib_info->camtolidarqw;
      // 内参
      calib_yaml["Sensor"]["Camera"]["intrinsic"]["model"] = "Pinhole";
      calib_yaml["Sensor"]["Camera"]["intrinsic"]["int_matrix"] = std::vector<double>{
          device_calib_info->camfx, 0.0, device_calib_info->camcx,
          0.0, device_calib_info->camfy, device_calib_info->camcy,
          0.0, 0.0, 1.0
      };
      calib_yaml["Sensor"]["Camera"]["intrinsic"]["dist_coeff"] = std::vector<double>{
          device_calib_info->camdistcoeff1, device_calib_info->camdistcoeff2, device_calib_info->camdistcoeff3, device_calib_info->camdistcoeff4,
          device_calib_info->camdistcoeff5, device_calib_info->camdistcoeff6, device_calib_info->camdistcoeff7, device_calib_info->camdistcoeff8
      };
      calib_yaml["Sensor"]["Camera"]["intrinsic"]["image_size"] = std::vector<double>{1280, 960};

      // 保存右相机参数
      // 外参
      calib_yaml["Sensor"]["Camera_R"]["base_link"] = "camera";
      calib_yaml["Sensor"]["Camera_R"]["frame"] = "camera_r";
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["x"] = device_calib_info->camrtocamtx;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["y"] = device_calib_info->camrtocamty;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["z"] = device_calib_info->camrtocamtz;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["x"] = device_calib_info->camrtocamqx;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["y"] = device_calib_info->camrtocamqy;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["z"] = device_calib_info->camrtocamqz;
      calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["w"] = device_calib_info->camrtocamqw;
      
      // 内参
      calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["model"] = "Pinhole";
      calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["int_matrix"] = std::vector<double>{
          device_calib_info->camrfx, 0.0, device_calib_info->camrcx,
          0.0, device_calib_info->camrfy, device_calib_info->camrcy,
          0.0, 0.0, 1.0
      };
      calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["dist_coeff"] = std::vector<double>{
          device_calib_info->camrdistcoeff1, device_calib_info->camrdistcoeff2, device_calib_info->camrdistcoeff3, device_calib_info->camrdistcoeff4,
          device_calib_info->camrdistcoeff5, device_calib_info->camrdistcoeff6, device_calib_info->camrdistcoeff7, device_calib_info->camrdistcoeff8
      };
      calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["image_size"] = std::vector<double>{1280, 960};

      // 保存IMU外参
      calib_yaml["Sensor"]["IMU"]["base_link"] = "sensor";
      calib_yaml["Sensor"]["IMU"]["frame"] = "imu";
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["x"] = device_calib_info->imutolidartx;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["y"] = device_calib_info->imutolidarty;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["z"] = device_calib_info->imutolidartz;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["x"] = device_calib_info->imutolidarqx;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["y"] = device_calib_info->imutolidarqy;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["z"] = device_calib_info->imutolidarqz;
      calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["w"] = device_calib_info->imutolidarqw;                    
      
      // 保存标定信息到文件
      calib_file_ = std::string("calibration.yaml");
      std::ofstream ofs_calib_file(calib_file_);
      if (ofs_calib_file) {
          ofs_calib_file << calib_yaml;
          ofs_calib_file.close();
          ROS_INFO("Calibration parameters saved to: %s", calib_file_.c_str());
      } else {
          ROS_ERROR("Failed to save calibration parameters to: %s", calib_file_.c_str());
          return;
      }
    }
  }

  void InitializeROS() {
    // Use message_filters for time synchronization
    nh_private_.param("queue_size", queue_size_, 10);
    nh_private_.param("time_sync_threshold", time_sync_threshold_, 0.01);
    nh_private_.param("publish_rectified", publish_rectified_, true);
    nh_private_.param("calib_file", calib_file_, std::string(""));
    nh_private_.param("config_file", config_file_, std::string("/workspace/ros_wrapper/lib_depth_inference/config/config.yaml"));
    nh_private_.param("left_image_topic", left_image_topic_, std::string("/rs_camera/left/color/image_raw"));
    nh_private_.param("right_image_topic", right_image_topic_, std::string("/rs_camera/right/color/image_raw"));
    nh_private_.param("disparity_topic", disparity_topic_, std::string("/stereo/disparity"));
    nh_private_.param("rectified_left_topic", rectified_left_topic_, std::string("/stereo/left/rectified"));
    nh_private_.param("rectified_right_topic", rectified_right_topic_, std::string("/stereo/right/rectified"));
    nh_private_.param("pointcloud_topic", pointcloud_topic_, std::string("/stereo/dense/pointcloud"));
    LoadCropParam();
    nh_private_.param("save_results", save_results_, false);
    nh_private_.param("save_results_path", save_results_path_, std::string(""));
    nh_private_.param("use_compressed_image", use_compressed_image_, false);

    if(calib_file_.empty()){
      GetDeviceCalibrationInfo();
    }
    ROS_INFO("config : %s, %s", config_file_.c_str(), calib_file_.c_str());

    depth_inference_ = robosense::ac_depth::createDepthEstimator();
    if (!depth_inference_) {
        ROS_ERROR("Failed to create depth estimator.");
        return;
    }     

    // Approximate time synchronization policy
    if(use_compressed_image_){
      left_image_topic_compressed_ = left_image_topic_ + "/compressed";
      right_image_topic_compressed_ = right_image_topic_ + "/compressed";
      // Subscribe to compressed image topics
      left_image_sub_compressed_.subscribe(nh_, left_image_topic_compressed_, queue_size_);
      right_image_sub_compressed_.subscribe(nh_, right_image_topic_compressed_, queue_size_);
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> SyncPolicyCompressed;
      sync_compressed_ = std::make_shared<message_filters::Synchronizer<SyncPolicyCompressed>>(SyncPolicyCompressed(queue_size_), left_image_sub_compressed_, right_image_sub_compressed_);
      sync_compressed_->registerCallback(boost::bind(&StereoNode::CompressedStereoImageCallback, this, _1, _2));
    } else {
        left_image_sub_.subscribe(nh_, left_image_topic_, queue_size_);
        right_image_sub_.subscribe(nh_, right_image_topic_, queue_size_);  
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), left_image_sub_, right_image_sub_);    
        sync_->registerCallback(boost::bind(&StereoNode::StereoImageCallback, this, _1, _2));
    }  
    // Publish topics
    disparity_pub_ = nh_.advertise<sensor_msgs::Image>(disparity_topic_, 1);
    
    if (publish_rectified_) {
      rectified_left_pub_ = nh_.advertise<sensor_msgs::Image>(rectified_left_topic_, 1);
      rectified_right_pub_ = nh_.advertise<sensor_msgs::Image>(rectified_right_topic_, 1);
    }
    
    // if (publish_pointcloud_) {
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_, 1);
    // }
    
    ROS_INFO("ROS topics initialized:");
    ROS_INFO("  Subscribing: %s, %s", left_image_topic_.c_str(), right_image_topic_.c_str());
    ROS_INFO("  Publishing: %s", disparity_topic_.c_str());
    if (publish_pointcloud_) {
      ROS_INFO("  Publishing pointcloud: %s", pointcloud_topic_.c_str());
    }
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  void StereoImageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg) {
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    uint64_t time_image_ns = left_msg->header.stamp.toNSec();
    // ROS_INFO("syncedCallback %lu.", time_image_ns);
    // Convert ROS image messages to OpenCV
    try {
      // Keep original encoding with passthrough, then convert to BGR8
      cv_ptr_left = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::RGB8);
      cv_ptr_right = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::RGB8);

      cv::Mat image_left_raw = cv_ptr_left->image;
      cv::Mat image_right_raw = cv_ptr_right->image;

      // Check image sizes
      if (image_left_raw.size() != image_right_raw.size()) {
        ROS_WARN("Left and right image sizes do not match!");
        return;
      }
      int img_cols_raw = image_left_raw.cols;
      int img_rows_raw = image_left_raw.rows;
      cv::Rect crop_roi = cv::Rect(crop_img_[2], crop_img_[0], img_cols_raw - crop_img_[2] - crop_img_[3], img_rows_raw - crop_img_[1] - crop_img_[0]);
      cv::Mat image_left = image_left_raw(crop_roi).clone();
      cv::Mat image_right = image_right_raw(crop_roi).clone();

      robosense::ac_depth::StereoImage image_data;
      int img_cols = image_left.cols;
      int img_rows = image_left.rows;            
      size_t left_image_size = image_left.total() * image_left.elemSize();
      size_t right_image_size = image_right.total() * image_right.elemSize();
      image_data.left_img_buffer = std::make_shared<std::vector<unsigned char>>(left_image_size);
      image_data.right_img_buffer = std::make_shared<std::vector<unsigned char>>(right_image_size);
      std::memcpy(image_data.left_img_buffer->data(), image_left.data, left_image_size);
      std::memcpy(image_data.right_img_buffer->data(), image_right.data, right_image_size);
      image_data.img_channel = image_left.channels();
      image_data.img_width = image_left.cols;
      image_data.img_height = image_left.rows;
      // image_data.timestamp = time_image * 1e9;
      image_data.timestamp = time_image_ns;

      robosense::ac_depth::LidarData lidar_data;
      // input data to depth estimation algorithm
      depth_inference_->onDataReceived(lidar_data, image_data);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }    
   
  } 
  
  void CompressedStereoImageCallback(const sensor_msgs::CompressedImageConstPtr& left_msg, const sensor_msgs::CompressedImageConstPtr& right_msg) {
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    uint64_t time_image_ns = left_msg->header.stamp.toNSec();
    // ROS_INFO("syncedCallback %lu.", time_image_ns);
    // Convert ROS image messages to OpenCV
    try {
      // Keep original encoding with passthrough, then convert to BGR8
      cv_ptr_left = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::RGB8);
      cv_ptr_right = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::RGB8);

      cv::Mat image_left_raw = cv_ptr_left->image;
      cv::Mat image_right_raw = cv_ptr_right->image;

      // Check image sizes
      if (image_left_raw.size() != image_right_raw.size()) {
        ROS_WARN("Left and right image sizes do not match!");
        return;
      }
      int img_cols_raw = image_left_raw.cols;
      int img_rows_raw = image_left_raw.rows;
      cv::Rect crop_roi = cv::Rect(crop_img_[2], crop_img_[0], img_cols_raw - crop_img_[2] - crop_img_[3], img_rows_raw - crop_img_[1] - crop_img_[0]);
      cv::Mat image_left = image_left_raw(crop_roi).clone();
      cv::Mat image_right = image_right_raw(crop_roi).clone();

      robosense::ac_depth::StereoImage image_data;
      int img_cols = image_left.cols;
      int img_rows = image_left.rows;            
      size_t left_image_size = image_left.total() * image_left.elemSize();
      size_t right_image_size = image_right.total() * image_right.elemSize();
      image_data.left_img_buffer = std::make_shared<std::vector<unsigned char>>(left_image_size);
      image_data.right_img_buffer = std::make_shared<std::vector<unsigned char>>(right_image_size);
      std::memcpy(image_data.left_img_buffer->data(), image_left.data, left_image_size);
      std::memcpy(image_data.right_img_buffer->data(), image_right.data, right_image_size);
      image_data.img_channel = image_left.channels();
      image_data.img_width = image_left.cols;
      image_data.img_height = image_left.rows;
      // image_data.timestamp = time_image * 1e9;
      image_data.timestamp = time_image_ns;

      robosense::ac_depth::LidarData lidar_data;
      // input data to depth estimation algorithm
      depth_inference_->onDataReceived(lidar_data, image_data);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }   
  } 
  
  // Inference result callback
  void ResultHandler(const std::shared_ptr<robosense::ac_depth::DepthInferenceResults>& results)
  {
    if (results) {
      if (!results->depth_image.depth_buffer || results->depth_image.depth_buffer->empty()) {
          ROS_ERROR("Callback Error: Depth buffer is NULL or empty.");
          return;
      }
      // Publish rectified rgb image            
      cv::Mat left_img_mat = cv::Mat(results->stereo_image.img_height, results->stereo_image.img_width, CV_8UC3, results->stereo_image.left_img_buffer->data());
      std_msgs::Header header;
      header.stamp = ros::Time(results->stereo_image.timestamp * 1.0e-9); // Convert ns to s
      cv_bridge::CvImage left_cv_img(header, "rgb8", left_img_mat);
      sensor_msgs::Image left_msg;
      left_cv_img.toImageMsg(left_msg);
      rectified_left_pub_.publish(left_msg);

      cv::Mat right_img_mat = cv::Mat(results->stereo_image.img_height, results->stereo_image.img_width, CV_8UC3, results->stereo_image.right_img_buffer->data());
      cv_bridge::CvImage right_cv_img(header, "rgb8", right_img_mat);
      sensor_msgs::Image right_msg;
      right_cv_img.toImageMsg(right_msg);
      rectified_right_pub_.publish(right_msg);

      // Publish depth image
      int depth_type = results->depth_image.bits_size == 32 ? CV_32FC1 : CV_16UC1;
      cv::Mat disp_img_mat = cv::Mat(results->depth_image.img_height, results->depth_image.img_width, depth_type, results->depth_image.depth_buffer->data());
      if(save_results_){
          std::string timestamp_str = std::to_string(results->depth_image.timestamp);
          timestamp_str = std::string(19 - timestamp_str.length(), '0') + timestamp_str;
          std::string disp_img_name = save_results_path_ + "/disparity/" + timestamp_str + ".tiff";
          cv::imwrite(disp_img_name, disp_img_mat);
          double focal_length = depth_inference_params_.fx;
          double baseline = depth_inference_params_.baseline;
          cv::Mat depth_img_mat = DisparityToDepth(disp_img_mat, focal_length, baseline);
          std::string depth_img_name = save_results_path_ + "/depth/" + timestamp_str + ".tiff";
          cv::imwrite(depth_img_name, depth_img_mat);          
          std::string left_img_name = save_results_path_ + "/left/" + timestamp_str + ".jpg";
          cv::Mat left_bgr;
          cv::cvtColor(left_img_mat, left_bgr, cv::COLOR_RGB2BGR);
          cv::imwrite(left_img_name, left_bgr);
      }

      cv::Mat disp_8u, disp_color, disp_16u;
      disp_img_mat.convertTo(disp_8u, CV_8UC1, 128);
      disp_img_mat.convertTo(disp_16u, CV_16UC1, 1000.0);
      cv::applyColorMap(disp_8u, disp_color, cv::COLORMAP_JET);
      cv::Mat mask = (disp_8u == 0); // Invalid depth mask
      disp_color.setTo(cv::Scalar(0, 0, 0), mask); // BGR black

      header.stamp = ros::Time(results->depth_image.timestamp * 1.0e-9); // Convert ns to s
      cv_bridge::CvImage disp_cv_img(header, "16UC1", disp_16u);
      sensor_msgs::Image depth_msg;            
      disp_cv_img.toImageMsg(depth_msg);
      disparity_pub_.publish(depth_msg);  
      
      // Publish color point cloud
      if (!results->color_point_cloud.point_cloud_buffer || 
          results->color_point_cloud.point_cloud_buffer->empty()) {
          ROS_WARN("Color point cloud buffer is empty, not publishing.");
          return;
      }
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl_point_cloud->header.stamp = results->color_point_cloud.timestamp / 1000; // us
      pcl_point_cloud->height = 1;
      pcl_point_cloud->is_dense = false;

      robosense::ac_depth::ColorPoint3D* color_point_cloud_ptr = (robosense::ac_depth::ColorPoint3D*)(results->color_point_cloud.point_cloud_buffer->data());
      pcl_point_cloud->points.reserve(results->color_point_cloud.point_num);
      for(size_t i = 0; i < results->color_point_cloud.point_num; i++) {
          robosense::ac_depth::ColorPoint3D color_pt = color_point_cloud_ptr[i];
          pcl::PointXYZRGBA pcl_pt;
          pcl_pt.x = color_pt.x;
          pcl_pt.y = color_pt.y;
          pcl_pt.z = color_pt.z;
          pcl_pt.r = color_pt.r;
          pcl_pt.g = color_pt.g;
          pcl_pt.b = color_pt.b;
          pcl_pt.a = color_pt.a;
          pcl_point_cloud->points.push_back(pcl_pt);
      }
      pcl_point_cloud->width = pcl_point_cloud->points.size();
      // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
      // pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
      // vg.setInputCloud(pcl_point_cloud);
      // vg.setLeafSize(0.01f, 0.01f, 0.01f);
      // vg.filter(*cloud_filtered);
      if(save_results_){
        std::string timestamp_str = std::to_string(results->color_point_cloud.timestamp);
        std::string pcd_name = save_results_path_ + "/points/" + timestamp_str + ".pcd";
        pcl::io::savePCDFile(pcd_name, *pcl_point_cloud);
      }
      sensor_msgs::PointCloud2 point_cloud_msg;
      pcl::toROSMsg(*pcl_point_cloud, point_cloud_msg);
      point_cloud_msg.header.frame_id = "rslidar";
      pointcloud_pub_.publish(point_cloud_msg);     
    } 
  }

  cv::Mat DisparityToDepth(const cv::Mat& disp, float f, float B) {
    cv::Mat disp32F;
    if (disp.type() != CV_32F) {
        disp.convertTo(disp32F, CV_32F);
    } else {
        disp32F = disp;
    }

    cv::Mat depth = cv::Mat::zeros(disp32F.size(), CV_32F);
    float fB = f * B;

    for (int i = 0; i < disp32F.rows; i++) {
        for (int j = 0; j < disp32F.cols; j++) {
            float d = disp32F.at<float>(i, j);
            
            // Filter invalid disparity (disparity <= 0 means invalid point or infinity)
            if (d > 0.0f) {
                depth.at<float>(i, j) = fB / d;
            } else {
                depth.at<float>(i, j) = 0.0f;
            }
        }
    }
    return depth;
}
  
private:
  // ROS-related
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  message_filters::Subscriber<sensor_msgs::CompressedImage> left_image_sub_compressed_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> right_image_sub_compressed_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> SyncPolicyCompressed;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyCompressed>> sync_compressed_;

  ros::Publisher disparity_pub_;
  ros::Publisher rectified_left_pub_;
  ros::Publisher rectified_right_pub_;
  ros::Publisher pointcloud_pub_;
  
  // Parameters
  std::string calib_file_;
  std::string config_file_;

  // Depth estimation inference object
  std::unique_ptr<robosense::ac_depth::DepthInferenceInterface> depth_inference_;
  robosense::ac_depth::DepthInferenceParams depth_inference_params_;

  // Topic names
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_image_topic_compressed_;
  std::string right_image_topic_compressed_;
  std::string disparity_topic_;
  std::string rectified_left_topic_;
  std::string rectified_right_topic_;
  std::string pointcloud_topic_;
  std::vector<int> crop_img_;
  bool publish_rectified_;
  bool publish_pointcloud_;
  bool save_results_;
  bool use_compressed_image_;
  std::string save_results_path_;
  // Queue size and time synchronization threshold
  int queue_size_;
  double time_sync_threshold_; 
  
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  StereoNode node(nh, nh_private);
  // node.Run();
  
  ros::spin();

  // node.Stop();

  return 0;
}

#elif ROS_VERSION == 2

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>


#include <cv_bridge/cv_bridge.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "depth_inference_api.h"
#include <robosense_msgs/msg/rs_ac_device_calib.hpp>

class StereoNode : public rclcpp::Node {
public:
  StereoNode() : rclcpp::Node("ros_node")
  {
    if (!InitializeROS()) {
      RCLCPP_ERROR(this->get_logger(), "ROS initialization failed.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Checking calibration file: %s", calib_file_.c_str());
    std::ifstream ifs_calib_file(calib_file_);
    if (!ifs_calib_file.good()) {
      RCLCPP_ERROR(this->get_logger(), "Sensor calibration file not found: %s", calib_file_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing depth inference engine...");
    if (!depth_inference_->initialize(config_file_, calib_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Depth inference initialization failed!");
      return;
    }

    depth_inference_params_ = depth_inference_->GetDepthInferenceParams();

    if (save_results_) {
      std::string model_name = depth_inference_params_.model_file.substr(
          depth_inference_params_.model_file.find_last_of("/") + 1);
      save_results_path_ = save_results_path_ + "/" + depth_inference_params_.version_num + "/" + model_name;
      std::filesystem::create_directories(save_results_path_ + "/disparity");
      std::filesystem::create_directories(save_results_path_ + "/depth");
      std::filesystem::create_directories(save_results_path_ + "/points");
      std::filesystem::create_directories(save_results_path_ + "/left");
      RCLCPP_INFO(this->get_logger(), "Save results to %s", save_results_path_.c_str());
    }

    depth_inference_->setResultsCallback(
        std::bind(&StereoNode::ResultHandler, this, std::placeholders::_1));
    depth_inference_->start();

    RCLCPP_INFO(this->get_logger(), "Stereo node initialized for ROS2.");
  }

  ~StereoNode() override
  {
    if (depth_inference_) {
      depth_inference_->stop();
    }
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using ImageConstSharedPtr = sensor_msgs::msg::Image::ConstSharedPtr;

  static uint64_t StampToNs(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<uint64_t>(rclcpp::Time(stamp).nanoseconds());
  }

  static builtin_interfaces::msg::Time NsToStamp(uint64_t ns)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
    return stamp;
  }

  static bool ParseCropString(const std::string &value, std::vector<int> &out)
  {
    out.clear();
    std::string cleaned = value;
    for (char &ch : cleaned) {
      if (!(std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+')) {
        ch = ' ';
      }
    }
    std::stringstream ss(cleaned);
    int v = 0;
    while (ss >> v) {
      out.push_back(v);
    }
    return out.size() == 4;
  }

  void LoadCropParam()
  {
    crop_img_ = {0, 0, 0, 0};
    this->declare_parameter("crop_img", std::vector<int64_t>{0, 0, 0, 0});

    rclcpp::Parameter crop_param;
    if (!this->get_parameter("crop_img", crop_param)) {
      RCLCPP_WARN(this->get_logger(), "crop_img not set. Using default [0, 0, 0, 0].");
      return;
    }

    std::vector<int> parsed;
    if (crop_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto raw = crop_param.as_integer_array();
      if (raw.size() == 4) {
        parsed.assign(raw.begin(), raw.end());
      }
    } else if (crop_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      ParseCropString(crop_param.as_string(), parsed);
    }

    if (parsed.size() == 4) {
      crop_img_ = parsed;
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid crop_img, expected 4 ints. Using default [0, 0, 0, 0].");
      crop_img_ = {0, 0, 0, 0};
    }
  }

  bool GetDeviceCalibrationInfo()
  {
    this->declare_parameter("device_calib_info_topic", std::string("/device_calib_info"));
    this->declare_parameter("device_calib_wait_sec", 20);
    const std::string device_calib_info_topic =
        this->get_parameter("device_calib_info_topic").as_string();
    const int wait_sec = this->get_parameter("device_calib_wait_sec").as_int();

    RCLCPP_INFO(this->get_logger(), "Waiting %d sec for RsACDeviceCalib on topic: %s",
                wait_sec, device_calib_info_topic.c_str());

    using CalibMsg = robosense_msgs::msg::RsACDeviceCalib;
    std::shared_ptr<CalibMsg> calib_msg;
    std::mutex calib_mutex;

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());
    auto calib_sub = this->create_subscription<CalibMsg>(
        device_calib_info_topic, rclcpp::SensorDataQoS(),
        [&](const CalibMsg::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(calib_mutex);
          calib_msg = msg;
          RCLCPP_INFO(this->get_logger(), "Received RsACDeviceCalib message");
        });

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(calib_mutex);
        if (calib_msg) {
          RCLCPP_INFO(this->get_logger(), "Got calibration message, breaking...");
          break;
        }
      }
      
      const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start).count();
      if (elapsed >= wait_sec) {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for calibration message");
        break;
      }
      
      // 处理待处理的事件
      exec.spin_some(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    exec.remove_node(this->get_node_base_interface());
    calib_sub.reset();

    if (!calib_msg) {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive device calibration info.");
      return false;
    }

    YAML::Node calib_yaml;
    calib_yaml["DEVICE_ID"] = calib_msg->header.frame_id;

    calib_yaml["Sensor"]["Lidar"]["base_link"] = "sensor";
    calib_yaml["Sensor"]["Lidar"]["frame"] = "lidar";
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["x"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["y"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["translation"]["z"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["x"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["y"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["z"] = 0.0;
    calib_yaml["Sensor"]["Lidar"]["extrinsic"]["quaternion"]["w"] = 1.0;

    calib_yaml["Sensor"]["Camera"]["base_link"] = "sensor";
    calib_yaml["Sensor"]["Camera"]["frame"] = "camera";
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["x"] = calib_msg->camtolidartx;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["y"] = calib_msg->camtolidarty;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["translation"]["z"] = calib_msg->camtolidartz;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["x"] = calib_msg->camtolidarqx;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["y"] = calib_msg->camtolidarqy;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["z"] = calib_msg->camtolidarqz;
    calib_yaml["Sensor"]["Camera"]["extrinsic"]["quaternion"]["w"] = calib_msg->camtolidarqw;
    calib_yaml["Sensor"]["Camera"]["intrinsic"]["model"] = "Pinhole";
    calib_yaml["Sensor"]["Camera"]["intrinsic"]["int_matrix"] = std::vector<double>{
        calib_msg->camfx, 0.0, calib_msg->camcx, 0.0, calib_msg->camfy, calib_msg->camcy, 0.0, 0.0, 1.0};
    calib_yaml["Sensor"]["Camera"]["intrinsic"]["dist_coeff"] = std::vector<double>{
        calib_msg->camdistcoeff1, calib_msg->camdistcoeff2, calib_msg->camdistcoeff3, calib_msg->camdistcoeff4,
        calib_msg->camdistcoeff5, calib_msg->camdistcoeff6, calib_msg->camdistcoeff7, calib_msg->camdistcoeff8};
    calib_yaml["Sensor"]["Camera"]["intrinsic"]["image_size"] = std::vector<double>{1280, 960};

    calib_yaml["Sensor"]["Camera_R"]["base_link"] = "camera";
    calib_yaml["Sensor"]["Camera_R"]["frame"] = "camera_r";
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["x"] = calib_msg->camrtocamtx;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["y"] = calib_msg->camrtocamty;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["translation"]["z"] = calib_msg->camrtocamtz;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["x"] = calib_msg->camrtocamqx;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["y"] = calib_msg->camrtocamqy;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["z"] = calib_msg->camrtocamqz;
    calib_yaml["Sensor"]["Camera_R"]["extrinsic"]["quaternion"]["w"] = calib_msg->camrtocamqw;
    calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["model"] = "Pinhole";
    calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["int_matrix"] = std::vector<double>{
        calib_msg->camrfx, 0.0, calib_msg->camrcx, 0.0, calib_msg->camrfy, calib_msg->camrcy, 0.0, 0.0, 1.0};
    calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["dist_coeff"] = std::vector<double>{
        calib_msg->camrdistcoeff1, calib_msg->camrdistcoeff2, calib_msg->camrdistcoeff3, calib_msg->camrdistcoeff4,
        calib_msg->camrdistcoeff5, calib_msg->camrdistcoeff6, calib_msg->camrdistcoeff7, calib_msg->camrdistcoeff8};
    calib_yaml["Sensor"]["Camera_R"]["intrinsic"]["image_size"] = std::vector<double>{1280, 960};

    calib_yaml["Sensor"]["IMU"]["base_link"] = "sensor";
    calib_yaml["Sensor"]["IMU"]["frame"] = "imu";
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["x"] = calib_msg->imutolidartx;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["y"] = calib_msg->imutolidarty;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["translation"]["z"] = calib_msg->imutolidartz;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["x"] = calib_msg->imutolidarqx;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["y"] = calib_msg->imutolidarqy;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["z"] = calib_msg->imutolidarqz;
    calib_yaml["Sensor"]["IMU"]["extrinsic"]["quaternion"]["w"] = calib_msg->imutolidarqw;

    calib_file_ = "calibration.yaml";
    std::ofstream ofs_calib_file(calib_file_);
    if (!ofs_calib_file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save calibration parameters to: %s", calib_file_.c_str());
      return false;
    }
    ofs_calib_file << calib_yaml;
    ofs_calib_file.close();
    RCLCPP_INFO(this->get_logger(), "Calibration parameters saved to: %s", calib_file_.c_str());
    return true;
  }

  bool InitializeROS()
  {
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("time_sync_threshold", 0.01);
    this->declare_parameter("publish_rectified", true);
    this->declare_parameter("calib_file", std::string(""));
    this->declare_parameter("config_file", std::string("/workspace/ros_wrapper/lib_depth_inference/config/config.yaml"));
    this->declare_parameter("left_image_topic", std::string("/rs_camera/left/color/image_raw"));
    this->declare_parameter("right_image_topic", std::string("/rs_camera/right/color/image_raw"));
    this->declare_parameter("disparity_topic", std::string("/stereo/disparity"));
    this->declare_parameter("rectified_left_topic", std::string("/stereo/left/rectified"));
    this->declare_parameter("rectified_right_topic", std::string("/stereo/right/rectified"));
    this->declare_parameter("pointcloud_topic", std::string("/stereo/dense/pointcloud"));
    this->declare_parameter("save_results", false);
    this->declare_parameter("save_results_path", std::string(""));

    queue_size_ = this->get_parameter("queue_size").as_int();
    time_sync_threshold_ = this->get_parameter("time_sync_threshold").as_double();
    publish_rectified_ = this->get_parameter("publish_rectified").as_bool();
    calib_file_ = this->get_parameter("calib_file").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    left_image_topic_ = this->get_parameter("left_image_topic").as_string();
    right_image_topic_ = this->get_parameter("right_image_topic").as_string();
    disparity_topic_ = this->get_parameter("disparity_topic").as_string();
    rectified_left_topic_ = this->get_parameter("rectified_left_topic").as_string();
    rectified_right_topic_ = this->get_parameter("rectified_right_topic").as_string();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    save_results_ = this->get_parameter("save_results").as_bool();
    save_results_path_ = this->get_parameter("save_results_path").as_string();

    LoadCropParam();

    if (calib_file_.empty()) {
      if (!GetDeviceCalibrationInfo()) {
        return false;
      }
    }

    depth_inference_ = robosense::ac_depth::createDepthEstimator();
    if (!depth_inference_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create depth estimator.");
      return false;
    }

    auto qos = rclcpp::SensorDataQoS();
    left_sub_ = this->create_subscription<ImageMsg>(
        left_image_topic_, qos, std::bind(&StereoNode::OnLeftImage, this, std::placeholders::_1));
    right_sub_ = this->create_subscription<ImageMsg>(
        right_image_topic_, qos, std::bind(&StereoNode::OnRightImage, this, std::placeholders::_1));

    disparity_pub_ = this->create_publisher<ImageMsg>(disparity_topic_, rclcpp::QoS(10));
    if (publish_rectified_) {
      rectified_left_pub_ = this->create_publisher<ImageMsg>(rectified_left_topic_, rclcpp::QoS(10));
      rectified_right_pub_ = this->create_publisher<ImageMsg>(rectified_right_topic_, rclcpp::QoS(10));
    }
    pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "config: %s, %s", config_file_.c_str(), calib_file_.c_str());
    return true;
  }

  bool TryGetSyncedPairLocked(std::pair<ImageConstSharedPtr, ImageConstSharedPtr> &pair)
  {
    if (!latest_left_ || !latest_right_) {
      return false;
    }

    const int64_t t_left = static_cast<int64_t>(StampToNs(latest_left_->header.stamp));
    const int64_t t_right = static_cast<int64_t>(StampToNs(latest_right_->header.stamp));
    const int64_t dt = std::llabs(t_left - t_right);
    const int64_t threshold_ns = static_cast<int64_t>(time_sync_threshold_ * 1e9);

    if (dt > threshold_ns) {
      if (t_left < t_right) {
        latest_left_.reset();
      } else {
        latest_right_.reset();
      }
      return false;
    }

    pair = {latest_left_, latest_right_};
    latest_left_.reset();
    latest_right_.reset();
    return true;
  }

  void OnLeftImage(const ImageConstSharedPtr msg)
  {
    std::pair<ImageConstSharedPtr, ImageConstSharedPtr> pair;
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      latest_left_ = msg;
      if (!TryGetSyncedPairLocked(pair)) {
        return;
      }
    }
    ProcessStereoPair(pair.first, pair.second);
  }

  void OnRightImage(const ImageConstSharedPtr msg)
  {
    std::pair<ImageConstSharedPtr, ImageConstSharedPtr> pair;
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      latest_right_ = msg;
      if (!TryGetSyncedPairLocked(pair)) {
        return;
      }
    }
    ProcessStereoPair(pair.first, pair.second);
  }

  void ProcessStereoPair(const ImageConstSharedPtr &left_msg, const ImageConstSharedPtr &right_msg)
  {
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    const uint64_t time_image_ns = StampToNs(left_msg->header.stamp);

    try {
      cv_ptr_left = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::RGB8);
      cv_ptr_right = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::RGB8);

      cv::Mat image_left_raw = cv_ptr_left->image;
      cv::Mat image_right_raw = cv_ptr_right->image;
      if (image_left_raw.size() != image_right_raw.size()) {
        RCLCPP_WARN(this->get_logger(), "Left and right image sizes do not match.");
        return;
      }

      const int img_cols_raw = image_left_raw.cols;
      const int img_rows_raw = image_left_raw.rows;
      const cv::Rect crop_roi(crop_img_[2], crop_img_[0],
                              img_cols_raw - crop_img_[2] - crop_img_[3],
                              img_rows_raw - crop_img_[1] - crop_img_[0]);
      cv::Mat image_left = image_left_raw(crop_roi).clone();
      cv::Mat image_right = image_right_raw(crop_roi).clone();

      robosense::ac_depth::StereoImage image_data;
      const size_t left_image_size = image_left.total() * image_left.elemSize();
      const size_t right_image_size = image_right.total() * image_right.elemSize();
      image_data.left_img_buffer = std::make_shared<std::vector<unsigned char>>(left_image_size);
      image_data.right_img_buffer = std::make_shared<std::vector<unsigned char>>(right_image_size);
      std::memcpy(image_data.left_img_buffer->data(), image_left.data, left_image_size);
      std::memcpy(image_data.right_img_buffer->data(), image_right.data, right_image_size);
      image_data.img_channel = image_left.channels();
      image_data.img_width = image_left.cols;
      image_data.img_height = image_left.rows;
      image_data.timestamp = time_image_ns;

      robosense::ac_depth::LidarData lidar_data;
      depth_inference_->onDataReceived(lidar_data, image_data);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  cv::Mat DisparityToDepth(const cv::Mat &disp, float f, float B)
  {
    cv::Mat disp32f;
    if (disp.type() != CV_32F) {
      disp.convertTo(disp32f, CV_32F);
    } else {
      disp32f = disp;
    }

    cv::Mat depth = cv::Mat::zeros(disp32f.size(), CV_32F);
    const float fB = f * B;
    for (int i = 0; i < disp32f.rows; ++i) {
      for (int j = 0; j < disp32f.cols; ++j) {
        const float d = disp32f.at<float>(i, j);
        depth.at<float>(i, j) = d > 0.0f ? (fB / d) : 0.0f;
      }
    }
    return depth;
  }

  void ResultHandler(const std::shared_ptr<robosense::ac_depth::DepthInferenceResults> &results)
  {
    if (!results || !results->depth_image.depth_buffer || results->depth_image.depth_buffer->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Callback error: depth buffer is null or empty.");
      return;
    }

    cv::Mat left_img_mat(results->stereo_image.img_height, results->stereo_image.img_width, CV_8UC3,
                         results->stereo_image.left_img_buffer->data());

    std_msgs::msg::Header header;
    header.stamp = NsToStamp(results->stereo_image.timestamp);

    if (publish_rectified_) {
      cv_bridge::CvImage left_cv_img(header, "rgb8", left_img_mat);
      ImageMsg left_msg;
      left_cv_img.toImageMsg(left_msg);
      rectified_left_pub_->publish(left_msg);

      cv::Mat right_img_mat(results->stereo_image.img_height, results->stereo_image.img_width, CV_8UC3,
                            results->stereo_image.right_img_buffer->data());
      cv_bridge::CvImage right_cv_img(header, "rgb8", right_img_mat);
      ImageMsg right_msg;
      right_cv_img.toImageMsg(right_msg);
      rectified_right_pub_->publish(right_msg);
    }

    const int depth_type = results->depth_image.bits_size == 32 ? CV_32FC1 : CV_16UC1;
    cv::Mat disp_img_mat(results->depth_image.img_height, results->depth_image.img_width, depth_type,
                         results->depth_image.depth_buffer->data());

    if (save_results_) {
      std::string timestamp_str = std::to_string(results->depth_image.timestamp);
      timestamp_str = std::string(19 - timestamp_str.length(), '0') + timestamp_str;
      cv::imwrite(save_results_path_ + "/disparity/" + timestamp_str + ".tiff", disp_img_mat);
      cv::imwrite(save_results_path_ + "/depth/" + timestamp_str + ".tiff",
                  DisparityToDepth(disp_img_mat, depth_inference_params_.fx, depth_inference_params_.baseline));
      cv::Mat left_bgr;
      cv::cvtColor(left_img_mat, left_bgr, cv::COLOR_RGB2BGR);
      cv::imwrite(save_results_path_ + "/left/" + timestamp_str + ".jpg", left_bgr);
    }

    cv::Mat disp_16u;
    disp_img_mat.convertTo(disp_16u, CV_16UC1, 1000.0);
    header.stamp = NsToStamp(results->depth_image.timestamp);
    cv_bridge::CvImage disp_cv_img(header, "16UC1", disp_16u);
    ImageMsg depth_msg;
    disp_cv_img.toImageMsg(depth_msg);
    disparity_pub_->publish(depth_msg);

    if (!results->color_point_cloud.point_cloud_buffer ||
        results->color_point_cloud.point_cloud_buffer->empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl_point_cloud->height = 1;
    pcl_point_cloud->is_dense = false;

    auto *color_point_cloud_ptr = reinterpret_cast<robosense::ac_depth::ColorPoint3D *>(
        results->color_point_cloud.point_cloud_buffer->data());
    pcl_point_cloud->points.reserve(results->color_point_cloud.point_num);
    for (size_t i = 0; i < results->color_point_cloud.point_num; ++i) {
      const auto &color_pt = color_point_cloud_ptr[i];
      pcl::PointXYZRGBA pcl_pt;
      pcl_pt.x = color_pt.x;
      pcl_pt.y = color_pt.y;
      pcl_pt.z = color_pt.z;
      pcl_pt.r = color_pt.r;
      pcl_pt.g = color_pt.g;
      pcl_pt.b = color_pt.b;
      pcl_pt.a = color_pt.a;
      pcl_point_cloud->points.push_back(pcl_pt);
    }
    pcl_point_cloud->width = pcl_point_cloud->points.size();

    if (save_results_) {
      const std::string pcd_name =
          save_results_path_ + "/points/" + std::to_string(results->color_point_cloud.timestamp) + ".pcd";
      pcl::io::savePCDFile(pcd_name, *pcl_point_cloud);
    }

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*pcl_point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "rslidar";
    point_cloud_msg.header.stamp = NsToStamp(results->color_point_cloud.timestamp);
    pointcloud_pub_->publish(point_cloud_msg);
  }

private:
  int queue_size_{10};
  double time_sync_threshold_{0.01};
  bool publish_rectified_{false};
  bool save_results_{false};

  std::string calib_file_;
  std::string config_file_;
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string disparity_topic_;
  std::string rectified_left_topic_;
  std::string rectified_right_topic_;
  std::string pointcloud_topic_;
  std::string save_results_path_;

  std::vector<int> crop_img_;

  std::unique_ptr<robosense::ac_depth::DepthInferenceInterface> depth_inference_;
  robosense::ac_depth::DepthInferenceParams depth_inference_params_;

  rclcpp::Subscription<ImageMsg>::SharedPtr left_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr right_sub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr disparity_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr rectified_left_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr rectified_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  std::mutex sync_mutex_;
  ImageConstSharedPtr latest_left_;
  ImageConstSharedPtr latest_right_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#else
#error "Unsupported ROS_VERSION. Expected 1 or 2."
#endif
