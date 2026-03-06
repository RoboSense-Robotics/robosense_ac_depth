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

#include <vector>
#include <memory>
#include <functional>
#include <cstring>
#include <string>
#include <utility>

namespace robosense{
namespace ac_depth{
// --- Data structure definitions ---
/**
 * @struct ColorPoint3D
 * @brief Colored 3D point
 *
 * Includes point coordinates and color
 */
struct ColorPoint3D
{
  float x;
  float y;
  float z;
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
  ColorPoint3D() = default;
  ~ColorPoint3D() = default;
};

/**
 * @struct LidarData
 * @brief Stores raw LiDAR data
 *
 * Raw data is stored as a byte stream, including key information such as timestamp and data size.
 */
struct LidarData
{
    std::shared_ptr<std::vector<unsigned char>> buffer;        // Pointer to LiDAR data buffer; memory is managed by the external caller
    unsigned int size = 0;                            // Buffer size in bytes
    unsigned long long timestamp = 0;                 // Data timestamp in ns
    LidarData() = default; 
    ~LidarData() = default;
};

/**
 * @struct StereoImage
 * @brief Stores stereo camera image data
 *
 * Contains left/right image data, image format, dimensions, and timestamp.
 */
struct StereoImage
{
    std::shared_ptr<std::vector<unsigned char>>left_img_buffer;    // Pointer to left image data buffer
    std::shared_ptr<std::vector<unsigned char>>right_img_buffer;   // Pointer to right image data buffer
    unsigned int img_channel = 0;                       // Number of image channels (e.g., 1 for grayscale, 3 for RGB)
    unsigned int img_width = 0;                         // Image width
    unsigned int img_height = 0;                        // Image height
    unsigned long long timestamp = 0;                   // Image timestamp in ns
    StereoImage() = default;    
    ~StereoImage() = default;
};

/**
 * @struct DepthImage
 * @brief Stores depth image data
 *
 * Depth image is one of the algorithm outputs, including data format and dimensions.
 */
struct DepthImage
{
    std::shared_ptr<std::vector<unsigned char>>depth_buffer;      // Pointer to depth image data buffer, depth unit is meters
    unsigned int bits_size = 0;                          // Number of bits per depth pixel
    unsigned int img_width = 0;                          // Depth image width
    unsigned int img_height = 0;                         // Depth image height
    unsigned long long timestamp = 0;                    // Depth image timestamp in ns
    DepthImage() = default;
    ~DepthImage() = default;
};

/**
 * @struct ColorPointCloud
 * @brief Stores colored point cloud data
 *
 * Contains a point cloud data buffer where each point includes X, Y, Z and R, G, B color values.
 */
struct ColorPointCloud {
    std::shared_ptr<std::vector<unsigned char>> point_cloud_buffer; // Pointer to colored point cloud data buffer
    unsigned int point_num = 0;                            // Number of points in the cloud
    unsigned int point_step = 0;                           // Bytes per point (X,Y,Z,R,G,B)
    unsigned long long timestamp = 0;                      // Point cloud timestamp in ns
    ColorPointCloud() = default;
    ~ColorPointCloud() = default;
};

/**
 * @struct DepthInferenceStatus
 * @brief Algorithm runtime status parameters
 *
 * Status parameters indicate the current algorithm state and are returned with each callback result.
 */

struct DepthInferenceStatus{
    int running_status;         // Algorithm running status flag
    int sync_status;            // Data synchronization status flag
    double time_delay;          // Delay of current frame
};

/**
 * @struct DepthInferenceResults
 * @brief Encapsulates all outputs of the depth estimation algorithm
 *
 * Returns all related results in a single callback.
 */
struct DepthInferenceResults {
    StereoImage stereo_image;            // Input stereo image
    LidarData lidar_data;                // Synchronized LiDAR data
    DepthImage depth_image;              // Algorithm output: depth estimation result
    ColorPointCloud color_point_cloud;   // Algorithm output: colored point cloud result
    unsigned long long ret_timestamp;    // Algorithm result timestamp in ns
    unsigned long long sys_timestamp;    // System timestamp in ns
    DepthInferenceStatus status;        // Algorithm status
    DepthInferenceResults() = default;
    ~DepthInferenceResults() = default;
};

struct DepthInferenceParams{
    std::string version_num;
    std::string model_file;
    double fx, fy, cx, cy;
    double baseline;
};

// --- Callback type definition ---

/**
 * @using DepthInferenceCallback
 * @brief Defines the callback function type
 *
 * This function is called after algorithm processing completes.
 * It receives a smart pointer to a DepthInferenceResults struct.
 */
using DepthInferenceCallback = std::function<void(const std::shared_ptr<DepthInferenceResults>&)>;

class DepthInferenceInterface {
public:
    DepthInferenceInterface() = default;
    virtual ~DepthInferenceInterface() {};

    // 1. Read initialization files
    // @param config_params_file: Path to configuration parameter file
    // @param sensor_calibration_file: Path to sensor calibration file
    // @return: bool, true indicates success, false indicates failure
    virtual bool initialize(const std::string& config_params_file, const std::string& sensor_calibration_file) = 0;

    // 2. Set callback function
    // @param callback: Function pointer of callback function
    // @return: bool, true indicates success, false indicates failure
    virtual bool setResultsCallback(DepthInferenceCallback callback) = 0;

    // 3. Receive and process LiDAR data and stereo images
    // This interface receives data and triggers algorithm processing
    // @param lidar_data: LiDAR data
    // @param image_data: Stereo image data
    virtual void onDataReceived(const LidarData& lidar_data, const StereoImage& image_data) = 0;

    // 4. Start algorithm processing
    // @brief Starts all internal processing, threads, and data reception.
    // @return: bool, true indicates success, false indicates failure
    virtual bool start() = 0;

    // 5. Stop algorithm processing
    // @brief Stops all internal processing, threads, and data reception.
    // @return: bool, true indicates successful stop, false indicates stop failure or already stopped.
    virtual bool stop() = 0;

    // 6. Get algorithm parameters
    // @brief Gets internal parameters of the inference module
    // @return: DepthInferenceParams: returns the parameter struct.
    virtual DepthInferenceParams GetDepthInferenceParams() = 0;
};

// --- Factory function declaration for creating algorithm instances ---
std::unique_ptr<DepthInferenceInterface> createDepthEstimator();

} // namespace ac_depth
} //namespace robosense
