/************************************************************************
Copyright 2026 RoboSense Technology Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
************************************************************************/

#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

namespace stereo_base {

/**
 * @brief Camera intrinsic parameter struct
 */
struct CameraIntrinsic {
  std::string model;                    // Camera model (Pinhole)
  cv::Mat camera_matrix;                // 3x3 intrinsic matrix
  cv::Mat dist_coeffs;                  // Distortion coefficients
  cv::Size image_size;                  // Image size
};

/**
 * @brief Camera extrinsic parameter struct
 */
struct CameraExtrinsic {
  cv::Vec3d translation;                // Translation vector
  cv::Vec4d quaternion;                 // Quaternion (x, y, z, w)
  cv::Mat rotation_matrix;              // 3x3 rotation matrix
  cv::Mat tvec;                         // 3x1 translation vector
};

/**
 * @brief Stereo calibration data struct
 */
struct StereoCalibration {
  CameraIntrinsic left_intrinsic;
  CameraIntrinsic right_intrinsic;
  CameraExtrinsic left_extrinsic;      // Extrinsics of Camera relative to sensor
  CameraExtrinsic right_extrinsic;     // Extrinsics of Camera_R relative to sensor
  CameraExtrinsic stereo_extrinsic;    // Extrinsics of Camera_R relative to Camera (computed)
  
  // Calibration quality metrics
  double stereo_rms_error;
  double stereo_epipolar_error;
  
  bool is_valid;
};

/**
 * @brief Stereo calibration file loader
 */
class StereoCalibrationLoader {
public:
  StereoCalibrationLoader() = default;
  ~StereoCalibrationLoader() = default;

  /**
   * @brief Load calibration data from YAML file
   * @param yaml_path Calibration file path
   * @return Stereo calibration data struct
   */
  StereoCalibration LoadFromYAML(const std::string& yaml_path);

private:
  /**
   * @brief Parse camera intrinsics
   */
  CameraIntrinsic ParseIntrinsic(const RS_YAML::Node& camera_node);

  /**
   * @brief Parse camera extrinsics
   */
  CameraExtrinsic ParseExtrinsic(const RS_YAML::Node& camera_node);

  /**
   * @brief Convert quaternion to rotation matrix
   */
  cv::Mat QuaternionToRotationMatrix(const cv::Vec4d& quaternion);

  /**
   * @brief Compute relative extrinsics between two cameras
   * @param left_ext Left camera extrinsics relative to reference
   * @param right_ext Right camera extrinsics relative to reference
   * @return Right camera extrinsics relative to left camera
   */
  CameraExtrinsic ComputeRelativeExtrinsic(const CameraExtrinsic& left_ext,
                                           const CameraExtrinsic& right_ext);
};

} // namespace stereo_base
