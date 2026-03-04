#pragma once

#include "common/stereo_calibration.hpp"
#include <opencv2/opencv.hpp>

namespace stereo_base {

/**
 * @brief Stereo image rectification class
 * Responsible for undistortion and epipolar rectification of stereo camera images
 */
class StereoRectifier {
public:
  StereoRectifier() = default;
  ~StereoRectifier() = default;

  /**
   * @brief Initialize rectification maps
   * @param calib Stereo calibration data
   * @return Whether initialization succeeds
   */
  bool Initialize(const StereoCalibration& calib);

  /**
   * @brief Rectify a stereo image pair
   * @param left_image Left image input
   * @param right_image Right image input
   * @param left_rectified Rectified left image output
   * @param right_rectified Rectified right image output
   */
  void Rectify(const cv::Mat& left_image, const cv::Mat& right_image,
               cv::Mat& left_rectified, cv::Mat& right_rectified);

  /**
   * @brief Get rectified camera intrinsics
   */
  cv::Mat GetRectifiedCameraMatrix() const { return P1_; }

  /**
   * @brief Get baseline length (unit: meters)
   */
  double GetBaseline() const { return baseline_; }

  /**
   * @brief Whether it is initialized
   */
  bool IsInitialized() const { return initialized_; }

  cv::Mat GetQMatrix() const { return Q_; }

  cv::Mat GetRectifiedLeftCameraMatrix() const { return P1_; }

  cv::Mat GetRectifiedRightCameraMatrix() const { return P2_; }

private:
  bool initialized_ = false;

  // Original calibration data
  StereoCalibration calib_;

  // Stereo rectification parameters
  cv::Mat R1_, R2_;        // Rectification rotation matrices for left/right cameras
  cv::Mat P1_, P2_;        // Projection matrices for left/right cameras
  cv::Mat Q_;              // Disparity-to-depth conversion matrix
  cv::Mat map1_left_, map2_left_;     // Left camera remap
  cv::Mat map1_right_, map2_right_;   // Right camera remap

  double baseline_;        // Baseline length
  cv::Size image_size_;    // Image size
};

} // namespace stereo_base
