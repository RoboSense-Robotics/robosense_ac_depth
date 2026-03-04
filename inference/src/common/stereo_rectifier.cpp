#include "common/stereo_rectifier.hpp"
#include <iostream>

namespace stereo_base {

bool StereoRectifier::Initialize(const StereoCalibration& calib) {
  if (!calib.is_valid) {
    std::cerr << "[StereoRectifier] Invalid calibration data" << std::endl;
    return false;
  }

  calib_ = calib;
  image_size_ = calib.left_intrinsic.image_size;

  try {
    // Get intrinsic matrices and distortion coefficients of left/right cameras
    cv::Mat K1 = calib.left_intrinsic.camera_matrix;
    cv::Mat D1 = calib.left_intrinsic.dist_coeffs;
    cv::Mat K2 = calib.right_intrinsic.camera_matrix;
    cv::Mat D2 = calib.right_intrinsic.dist_coeffs;

    // Get stereo extrinsics (right camera relative to left camera)
    cv::Mat R = calib.stereo_extrinsic.rotation_matrix;
    cv::Mat T = calib.stereo_extrinsic.tvec;
    cv::Mat R_inv = R.inv();
    cv::Mat T_inv = -R_inv * T;
    // Compute baseline length
    baseline_ = cv::norm(T);

    std::cout << "[StereoRectifier] Initializing stereo rectification..." << std::endl;
    std::cout << "  Image size: " << image_size_ << std::endl;
    std::cout << "  Baseline: " << baseline_ << " meters" << std::endl;
    
    // Print epipolar rectification inputs
    std::cout << "[StereoRectifier] Input K1:" << std::endl << K1 << std::endl;
    std::cout << "[StereoRectifier] Input K2:" << std::endl << K2 << std::endl;
    std::cout << "[StereoRectifier] Input D1:" << std::endl << D1 << std::endl;
    std::cout << "[StereoRectifier] Input D2:" << std::endl << D2 << std::endl;
    std::cout << "[StereoRectifier] R:" << std::endl << R << std::endl;
    std::cout << "[StereoRectifier] T:" << std::endl << T << std::endl;
        
    cv::Rect validRoi[2];
    // Perform stereo rectification
    cv::stereoRectify(
        K1, D1,           // Left camera intrinsics and distortion
        K2, D2,           // Right camera intrinsics and distortion
        image_size_,      // Image size
        R_inv, T_inv,     // Stereo extrinsics
        R1_, R2_,         // Output: rectification rotation matrices
        P1_, P2_,         // Output: projection matrices
        Q_,               // Output: disparity-to-depth conversion matrix
        cv::CALIB_ZERO_DISPARITY,  // Flag: align rectified image pairs
        0,                // alpha: 0 keeps only valid pixels
        image_size_       // Output image size
    );
        
    std::cout << "[StereoRectifier] Output P1:" << std::endl << P1_ << std::endl;
    std::cout << "[StereoRectifier] Output P2:" << std::endl << P2_ << std::endl;
    std::cout << "[StereoRectifier] Output Q:" << std::endl << Q_ << std::endl;
    std::cout << "[StereoRectifier] Valid ROI[0]: " << validRoi[0] << std::endl;
    std::cout << "[StereoRectifier] Valid ROI[1]: " << validRoi[1] << std::endl;

    // Compute remap tables
    cv::initUndistortRectifyMap(
        K1, D1, R1_, P1_,
        image_size_, CV_32FC1,
        map1_left_, map2_left_
    );

    cv::initUndistortRectifyMap(
        K2, D2, R2_, P2_,
        image_size_, CV_32FC1,
        map1_right_, map2_right_
    );

    initialized_ = true;
    std::cout << "[StereoRectifier] Stereo rectification initialized successfully" << std::endl;

    // Print rectified projection matrix info
    std::cout << "  Rectified P1: fx=" << P1_.at<double>(0, 0) 
              << ", fy=" << P1_.at<double>(1, 1)
              << ", cx=" << P1_.at<double>(0, 2)
              << ", cy=" << P1_.at<double>(1, 2) << std::endl;

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[StereoRectifier] Failed to initialize: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  }
}

void StereoRectifier::Rectify(const cv::Mat& left_image, const cv::Mat& right_image,
                               cv::Mat& left_rectified, cv::Mat& right_rectified) {
  if (!initialized_) {
    std::cerr << "[StereoRectifier] Not initialized!" << std::endl;
    left_rectified = cv::Mat();
    right_rectified = cv::Mat();
    return;
  }
  
  // Check input image size
  if (left_image.size() != image_size_ || right_image.size() != image_size_) {
    std::cerr << "[StereoRectifier] Image size mismatch! Expected: " << image_size_ 
              << ", Got left: " << left_image.size() 
              << ", right: " << right_image.size() << std::endl;
    left_rectified = cv::Mat();
    right_rectified = cv::Mat();
    return;
  }
  


  // Check whether input images are empty
  if (left_image.empty() || right_image.empty()) {
    std::cerr << "[StereoRectifier] Input images are empty!" << std::endl;
    left_rectified = cv::Mat();
    right_rectified = cv::Mat();
    return;
  }

  double min_val_input, max_val_input;
  cv::minMaxLoc(left_image, &min_val_input, &max_val_input);
  // ROS_INFO_THROTTLE(5.0, "Input left image - size: %dx%d, channels: %d, type: %d, min: %.2f, max: %.2f", 
  //                   left_image.cols, left_image.rows, left_image.channels(), 
  //                   left_image.type(), min_val_input, max_val_input);

  
  // Check whether remap tables are valid
  if (map1_left_.empty() || map2_left_.empty()) {
    std::cerr << "[StereoRectifier] Remap maps are empty!" << std::endl;
    left_rectified = cv::Mat();
    right_rectified = cv::Mat();
    return;
  }
  
  // std::cout << "[StereoRectifier] Rectifying - input size: " << left_image.size() 
  //           << ", map size: " << map1_left_.size() 
  //           << ", input type: " << left_image.type() 
  //           << ", map1 type: " << map1_left_.type() 
  //           << ", map2 type: " << map2_left_.type() << std::endl;
  
  // Check value range of remap tables
  double map1_min, map1_max, map2_min, map2_max;
  cv::minMaxLoc(map1_left_, &map1_min, &map1_max);
  cv::minMaxLoc(map2_left_, &map2_min, &map2_max);
  // std::cout << "[StereoRectifier] map1 range: [" << map1_min << ", " << map1_max << "]" << std::endl;
  // std::cout << "[StereoRectifier] map2 range: [" << map2_min << ", " << map2_max << "]" << std::endl;

  // Apply remap for undistortion and epipolar rectification
  cv::remap(left_image, left_rectified, map1_left_, map2_left_, cv::INTER_LINEAR);
  cv::remap(right_image, right_rectified, map1_right_, map2_right_, cv::INTER_LINEAR);
  
  // Immediately check output result
  double out_min, out_max;
  cv::minMaxLoc(left_rectified, &out_min, &out_max);
  // std::cout << "[StereoRectifier] Rectified - output size: " << left_rectified.size() 
  //           << ", output type: " << left_rectified.type() 
  //           << ", value range: [" << out_min << ", " << out_max << "]" << std::endl;
  
  if (out_max == 0) {
    std::cerr << "[StereoRectifier] WARNING: Output image is all black! Possible issues:" << std::endl;
    std::cerr << "  1. Remap maps may be invalid or out of bounds" << std::endl;
    std::cerr << "  2. Calibration parameters may be incorrect" << std::endl;
    std::cerr << "  3. Image size mismatch with calibration" << std::endl;
  }
}

} // namespace stereo_base
