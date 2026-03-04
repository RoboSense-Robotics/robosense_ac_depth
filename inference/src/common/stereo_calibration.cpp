#include "common/stereo_calibration.hpp"
#include <iostream>
#include <stdexcept>

namespace stereo_base {

StereoCalibration StereoCalibrationLoader::LoadFromYAML(const std::string& yaml_path) {
  StereoCalibration calib;
  calib.is_valid = false;

  try {
    RS_YAML::Node config = RS_YAML::LoadFile(yaml_path);

    // Check whether required nodes exist
    if (!config["Sensor"] || !config["Sensor"]["Camera"] || !config["Sensor"]["Camera_R"]) {
      throw std::runtime_error("Invalid calibration file format: missing Camera or Camera_R");
    }

    // Parse left camera (Camera) intrinsics and extrinsics
    RS_YAML::Node left_camera = config["Sensor"]["Camera"];
    calib.left_intrinsic = ParseIntrinsic(left_camera);
    calib.left_extrinsic = ParseExtrinsic(left_camera);

    // Parse right camera (Camera_R) intrinsics and extrinsics
    RS_YAML::Node right_camera = config["Sensor"]["Camera_R"];
    calib.right_intrinsic = ParseIntrinsic(right_camera);
    calib.right_extrinsic = ParseExtrinsic(right_camera);

    // Compute extrinsics of right camera relative to left camera (T_right_left = T_right_sensor * T_sensor_left)
    calib.stereo_extrinsic = calib.right_extrinsic;//ComputeRelativeExtrinsic(calib.left_extrinsic, calib.right_extrinsic);

    // Read calibration quality metrics
    if (config["CALI_INFO"] && config["CALI_INFO"]["CAM_INTRINSIC"]) {
      calib.stereo_rms_error = config["CALI_INFO"]["CAM_INTRINSIC"]["stereo_rms_error"].as<double>();
      calib.stereo_epipolar_error = config["CALI_INFO"]["CAM_INTRINSIC"]["stereo_epipolar_error"].as<double>();
    }

    calib.is_valid = true;
    std::cout << "[StereoCalibrationLoader] Successfully loaded calibration from: " << yaml_path << std::endl;
    std::cout << "  Left camera:  " << calib.left_intrinsic.image_size << std::endl;
    std::cout << "  Right camera: " << calib.right_intrinsic.image_size << std::endl;
    std::cout << "  Stereo RMS error: " << calib.stereo_rms_error << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "[StereoCalibrationLoader] Error loading calibration file: " << e.what() << std::endl;
    calib.is_valid = false;
  }

  return calib;
}

CameraIntrinsic StereoCalibrationLoader::ParseIntrinsic(const RS_YAML::Node& camera_node) {
  CameraIntrinsic intrinsic;

  if (!camera_node["intrinsic"]) {
    throw std::runtime_error("Missing intrinsic node");
  }

  RS_YAML::Node intrinsic_node = camera_node["intrinsic"];

  // Parse camera model
  intrinsic.model = intrinsic_node["model"].as<std::string>();

  // Parse intrinsic matrix (3x3)
  if (intrinsic_node["int_matrix"]) {
    std::vector<double> int_matrix = intrinsic_node["int_matrix"].as<std::vector<double>>();
    if (int_matrix.size() != 9) {
      throw std::runtime_error("Invalid intrinsic matrix size");
    }
    intrinsic.camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      intrinsic.camera_matrix.at<double>(i / 3, i % 3) = int_matrix[i];
    }
  }

  // Parse distortion coefficients
  if (intrinsic_node["dist_coeff"]) {
    std::vector<double> dist_coeff = intrinsic_node["dist_coeff"].as<std::vector<double>>();
    intrinsic.dist_coeffs = cv::Mat(dist_coeff, true);
  }

  // Parse image size
  if (intrinsic_node["image_size"]) {
    std::vector<int> image_size = intrinsic_node["image_size"].as<std::vector<int>>();
    if (image_size.size() == 2) {
      intrinsic.image_size = cv::Size(image_size[0], image_size[1]);
    }
  }

  return intrinsic;
}

CameraExtrinsic StereoCalibrationLoader::ParseExtrinsic(const RS_YAML::Node& camera_node) {
  CameraExtrinsic extrinsic;

  if (!camera_node["extrinsic"]) {
    throw std::runtime_error("Missing extrinsic node");
  }

  RS_YAML::Node extrinsic_node = camera_node["extrinsic"];

  // Parse translation vector
  if (extrinsic_node["translation"]) {
    double x = extrinsic_node["translation"]["x"].as<double>();
    double y = extrinsic_node["translation"]["y"].as<double>();
    double z = extrinsic_node["translation"]["z"].as<double>();
    extrinsic.translation = cv::Vec3d(x, y, z);
    extrinsic.tvec = (cv::Mat_<double>(3, 1) << x, y, z);
  }

  // Parse quaternion
  if (extrinsic_node["quaternion"]) {
    double x = extrinsic_node["quaternion"]["x"].as<double>();
    double y = extrinsic_node["quaternion"]["y"].as<double>();
    double z = extrinsic_node["quaternion"]["z"].as<double>();
    double w = extrinsic_node["quaternion"]["w"].as<double>();
    extrinsic.quaternion = cv::Vec4d(x, y, z, w);
    extrinsic.rotation_matrix = QuaternionToRotationMatrix(extrinsic.quaternion);
  }

  return extrinsic;
}

cv::Mat StereoCalibrationLoader::QuaternionToRotationMatrix(const cv::Vec4d& q) {
  // q = (x, y, z, w)
  double x = q[0], y = q[1], z = q[2], w = q[3];

  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  
  R.at<double>(0, 0) = 1 - 2*y*y - 2*z*z;
  R.at<double>(0, 1) = 2*x*y - 2*z*w;
  R.at<double>(0, 2) = 2*x*z + 2*y*w;
  
  R.at<double>(1, 0) = 2*x*y + 2*z*w;
  R.at<double>(1, 1) = 1 - 2*x*x - 2*z*z;
  R.at<double>(1, 2) = 2*y*z - 2*x*w;
  
  R.at<double>(2, 0) = 2*x*z - 2*y*w;
  R.at<double>(2, 1) = 2*y*z + 2*x*w;
  R.at<double>(2, 2) = 1 - 2*x*x - 2*y*y;

  return R;
}

CameraExtrinsic StereoCalibrationLoader::ComputeRelativeExtrinsic(
    const CameraExtrinsic& left_ext,
    const CameraExtrinsic& right_ext) {
  
  // T_left_sensor: transform of left camera relative to sensor
  // T_right_sensor: transform of right camera relative to sensor
  // Solve: T_right_left = T_right_sensor * inv(T_left_sensor)
  
  cv::Mat R_left = left_ext.rotation_matrix;
  cv::Mat t_left = left_ext.tvec;
  
  cv::Mat R_right = right_ext.rotation_matrix;
  cv::Mat t_right = right_ext.tvec;
  
  // Compute T_sensor_left = inv(T_left_sensor)
  // R_sensor_left = R_left^T
  // t_sensor_left = -R_left^T * t_left
  cv::Mat R_sensor_left = R_left.t();
  cv::Mat t_sensor_left = -R_sensor_left * t_left;
  
  // Compute T_right_left = T_right_sensor * T_sensor_left
  // R_right_left = R_right * R_sensor_left
  // t_right_left = R_right * t_sensor_left + t_right
  cv::Mat R_relative = R_right * R_sensor_left;
  cv::Mat t_relative = R_right * t_sensor_left + t_right;
  
  // Pack result
  CameraExtrinsic relative_ext;
  relative_ext.rotation_matrix = R_relative;
  relative_ext.tvec = t_relative;
  relative_ext.translation = cv::Vec3d(t_relative.at<double>(0),
                                       t_relative.at<double>(1),
                                       t_relative.at<double>(2));
  
  relative_ext.quaternion = cv::Vec4d(0, 0, 0, 1);
  
  return relative_ext;
}

} // namespace stereo_base
