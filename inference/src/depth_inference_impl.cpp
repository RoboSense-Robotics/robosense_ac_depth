#include "depth_inference_api.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <future>
#include <deque>
#include <chrono>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include "infer_base/common_utils/log.hpp"
#include "common/stereo_calibration.hpp"
#include "common/stereo_rectifier.hpp"
#include "yaml-cpp/yaml.h"

#ifdef ENABLE_TENSORRT
#include "infer_base/trt_core/trt_core.hpp"
#endif

#include "infer_stereo/InferenceStereo.hpp"
#include "infer_base/image_processing_utils/image_processing_utils.hpp"

namespace robosense {
namespace ac_depth {

class DepthInferenceImpl : public DepthInferenceInterface {
public:
    DepthInferenceImpl() {
        LOG_INFO("DepthInferenceImpl: Instance created.");
        m_debug_flag = 0;
        m_image_scale = 0.5;
    }

    ~DepthInferenceImpl() override {
        LOG_INFO("DepthInferenceImpl: Instance destroyed.");
        m_is_initialized = false;
        
        // Explicitly release model resources
        if (m_stereo_model) {
            m_stereo_model.reset();
            LOG_INFO("DepthInferenceImpl: Model released.");
        }
    }

    /**
     * @brief Initialize inference engine
     * @param config_params_file Path to configuration parameter file
     * @param sensor_calibration_file Path to sensor calibration file
     * @return true on success; false on failure
     */
    bool initialize(const std::string& config_params_file, const std::string& sensor_calibration_file) override {
        // Algorithm version
        m_algorithm_version_number = std::string(LIBDEPTH_VERSION);
        LOG_INFO("DepthInferenceImpl: algorithm version = %s", m_algorithm_version_number.c_str());

        if (m_is_initialized) {
            LOG_INFO("DepthInferenceImpl: Already initialized.");
            return true;
        }

        // 1. Read configuration file
        RS_YAML::Node config;
        try {
            config = RS_YAML::LoadFile(config_params_file);
        } catch (const RS_YAML::Exception& e) {
            LOG_ERROR("Failed to parse config file: %s", e.what());
            return false;
        }

        // 2. Parse configuration parameters
        try {
            m_config_version_number = config["version"] ? config["version"].as<std::string>() : "";
            m_model_path = config["model"]["file"] ? config["model"]["file"].as<std::string>() : "";
            m_model_name = config["model"]["name"] ? config["model"]["name"].as<std::string>() : "";
            m_mem_buf_size = config["model"]["mem_size"] ? config["model"]["mem_size"].as<int>() : 5;
            m_image_scale = config["model"]["image_scale"] ? config["model"]["image_scale"].as<double>() : 0.5;
            m_infer_img_height = int(960 * m_image_scale);
            m_infer_img_width = int(1280 * m_image_scale);
            m_debug_flag = config["debug_flag"] ? config["debug_flag"].as<int>() : 0;            
            // Load calibration file
            stereo_base::StereoCalibrationLoader loader;
            m_stereo_calibration = loader.LoadFromYAML(sensor_calibration_file);
            if (!m_stereo_calibration.is_valid) {
                LOG_ERROR("Invalid sensor calibration file.");
                return false;
            }
            config.reset();
        } catch (const RS_YAML::Exception& e) {
            LOG_ERROR("Failed to read config parameters: %s", e.what());
            return false;
        }

        // 3. Initialize rectifier
        LOG_INFO("Initializing stereo rectifier...");
        m_stereo_rectifier = std::make_shared<stereo_base::StereoRectifier>();
        if (!m_stereo_rectifier->Initialize(m_stereo_calibration)) {
            LOG_ERROR("Failed to initialize rectifier!");
            return false;
        }
        LOG_INFO("Stereo rectifier initialized. Baseline: %f m", m_stereo_rectifier->GetBaseline());

        // 4. Initialize inference engine
        LOG_INFO("Starting inference engine initialization...");
        LOG_INFO("  Model path: %s", m_model_path.c_str());
        LOG_INFO("  Input size: %d, %d" , m_infer_img_width, m_infer_img_height);
        LOG_INFO("  Memory buffer size: %d", m_mem_buf_size);
        
        if (!InitializeInferenceEngine()) {
            LOG_ERROR("Failed to initialize inference engine!");
            return false;
        }

        m_is_initialized = true;
        LOG_INFO("DepthInferenceImpl: Initialization complete.");
        return true;
    }

    /**
     * @brief Set result callback function
     * @param callback Callback function pointer
     * @return true on success; false on failure
     */
    bool setResultsCallback(DepthInferenceCallback callback) override {
        if (!callback) {
            LOG_ERROR("Callback function is null.");
            return false;
        }
        m_callback = callback;
        LOG_INFO("Callback function set successfully.");
        return true;
    }

    /**
     * @brief Receive data and submit inference task asynchronously
     * @param lidar_data LiDAR data
     * @param image_data Stereo image data
     */
    void onDataReceived(const LidarData& lidar_data, const StereoImage& image_data) override {
        // if (!m_is_initialized) {
        //     LOG_ERROR("Cannot process data. Not initialized.");
        //     return;
        // }

        // if (!m_callback) {
        //     LOG_ERROR("No callback function set.");
        //     return;
        // }

        // Validate input data
        if (image_data.img_width == 0 || image_data.img_height == 0 || 
            !image_data.left_img_buffer || !image_data.right_img_buffer) {
            LOG_ERROR("Invalid stereo image data.");
            return;
        }

        // Submit async task and immediately process completed tasks
        SubmitAsyncTask(lidar_data, image_data);
        ProcessCompletedTasks();
        
        if (m_async_tasks.size() > MAX_ASYNC_QUEUE_SIZE) {
            LOG_WARN("Async task queue size (%d) exceeds limit, dropping oldest task", m_async_tasks.size());
            m_async_tasks.pop_front();
        }

    }

    /**
     * @brief Start algorithm (async version does not require a separate thread)
     * @return true
     */
    bool start() override {
        if (!m_is_initialized) {
            LOG_ERROR("Cannot start. Not initialized.");
            return false;
        }
        LOG_INFO("Async inference ready.");
        return true;
    }

    /**
     * @brief Stop algorithm and clear all unfinished tasks
     * @return true
     */
    bool stop() override {
        LOG_INFO("Stopping async inference and clearing pending tasks...");
        
        // Clear all unfinished async tasks
        m_async_tasks.clear();
        
        m_is_initialized = false;
        LOG_INFO("Async inference stopped.");
        return true;
    }

    DepthInferenceParams GetDepthInferenceParams() override {
        DepthInferenceParams depth_inference_params;
        depth_inference_params.version_num = m_algorithm_version_number;
        depth_inference_params.model_file = m_model_path;
        depth_inference_params.baseline = m_stereo_rectifier->GetBaseline();
        cv::Mat P1 = m_stereo_rectifier->GetRectifiedLeftCameraMatrix();
        depth_inference_params.fx = P1.at<double>(0, 0) * m_image_scale;
        depth_inference_params.fy = P1.at<double>(1, 1) * m_image_scale;
        depth_inference_params.cx = P1.at<double>(0, 2) * m_image_scale;
        depth_inference_params.cy = P1.at<double>(1, 2) * m_image_scale;
        return depth_inference_params;
    }

private:
    bool m_is_initialized = false;
    DepthInferenceCallback m_callback = nullptr;  // Kept for backward compatibility
    
    // Configuration parameters
    std::string m_config_version_number;
    std::string m_algorithm_version_number;
    std::string m_model_path;
    std::string m_model_name;
    int m_mem_buf_size;
    double m_image_scale;
    uint64_t m_infer_img_height;
    uint64_t m_infer_img_width;
    int m_debug_flag;

    // Calibration and rectification
    stereo_base::StereoCalibration m_stereo_calibration;
    std::shared_ptr<stereo_base::StereoRectifier> m_stereo_rectifier;
    
    // Inference model
    std::shared_ptr<easy_deploy::BaseStereoMatchingModel> m_stereo_model;

    // Async inference task management
    struct AsyncInferenceTask {
        std::future<cv::Mat> disparity_future;
        LidarData lidar_data;
        StereoImage stereo_image;
        cv::Mat left_rectified;
        cv::Mat right_rectified;
        std::chrono::high_resolution_clock::time_point submit_time;
        std::chrono::high_resolution_clock::time_point rectify_time;
        std::chrono::high_resolution_clock::time_point infer_time;
    };
    
    std::deque<AsyncInferenceTask> m_async_tasks;
    static constexpr size_t MAX_ASYNC_QUEUE_SIZE = 10;

    /**
     * @brief Initialize inference engine
     */
    bool InitializeInferenceEngine() {
        LOG_INFO("Initializing inference engine...");

        try {
            bool need_normalize = false;
            bool need_to_rgb = true;
            #ifdef ENABLE_TENSORRT
                // Create TensorRT inference core
                auto engine = easy_deploy::CreateTrtInferCore(m_model_path);
                LOG_INFO("TensorRT engine created.");
            #else
                LOG_ERROR("No inference backend enabled (ENABLE_TENSORRT)");
                return false;
            #endif

            // Create preprocessing block
            auto preprocess_block = easy_deploy::CreateCpuImageProcessingResizePad(
                easy_deploy::ImageProcessingPadMode::TOP_RIGHT,
                easy_deploy::ImageProcessingPadValue::EDGE,
                need_to_rgb,     // to_rgb
                need_normalize,  // normalize
                {0.485, 0.456, 0.406},  // mean
                {0.229, 0.224, 0.225}   // std
            );
            m_stereo_model = easy_deploy::CreateInferenceStereoModel(
                engine,
                preprocess_block,
                m_infer_img_height,
                m_infer_img_width,
                {"left", "right"},
                {"disp"}
            );

            if (m_stereo_model) {
                LOG_INFO("Stereo model created successfully.");
                
            }

            // Initialize async inference pipeline
            m_stereo_model->InitPipeline();
            LOG_INFO("Async inference pipeline initialized.");

            #ifdef ENABLE_TENSORRT
                LOG_INFO("InferenceStereo TensorRT model initialized successfully.");
            #endif

            return true;

        } catch (const std::exception& e) {
            LOG_ERROR("Exception during model initialization: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Submit inference task asynchronously
     */
    void SubmitAsyncTask(const LidarData& lidar_data, const StereoImage& image_data) {
        const auto t0 = std::chrono::high_resolution_clock::now();

        // 1. Image undistortion and epipolar rectification
        cv::Mat left_img(image_data.img_height, image_data.img_width, CV_8UC3, 
                         image_data.left_img_buffer->data());
        cv::Mat right_img(image_data.img_height, image_data.img_width, CV_8UC3, 
                          image_data.right_img_buffer->data());
        
        cv::Mat left_rectified, right_rectified;
        m_stereo_rectifier->Rectify(left_img, right_img, left_rectified, right_rectified);

        const auto t1 = std::chrono::high_resolution_clock::now();
        auto d_rectify = 0.001 * (float)std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        // LOG_INFO("Time_Cost_Block_Name, rectify, cost(ms), %f", d_rectify);

        // 2. Resize rectified images to inference size
        cv::Mat left_resized, right_resized;
        cv::resize(left_rectified, left_resized, cv::Size(m_infer_img_width, m_infer_img_height));
        cv::resize(right_rectified, right_resized, cv::Size(m_infer_img_width, m_infer_img_height));

        // 3. Submit async inference task (use clone() to ensure data safety)
        std::future<cv::Mat> disparity_future = m_stereo_model->ComputeDispAsync(
            left_resized.clone(), right_resized.clone());

        if (!disparity_future.valid()) {
            LOG_WARN("Failed to submit async inference task!");
            return;
        }
        const auto t2 = std::chrono::high_resolution_clock::now();
        // 3. Store task in queue (save rectified images)
        AsyncInferenceTask task;
        task.disparity_future = std::move(disparity_future);
        task.lidar_data = lidar_data;
        task.stereo_image = image_data;
        // Replace original image buffers with rectified images
        size_t left_rect_size = left_rectified.total() * left_rectified.elemSize();
        size_t right_rect_size = right_rectified.total() * right_rectified.elemSize();
        task.stereo_image.left_img_buffer = std::make_shared<std::vector<uint8_t>>(left_rect_size);
        task.stereo_image.right_img_buffer = std::make_shared<std::vector<uint8_t>>(right_rect_size);
        std::memcpy(task.stereo_image.left_img_buffer->data(), left_rectified.data, left_rect_size);
        std::memcpy(task.stereo_image.right_img_buffer->data(), right_rectified.data, right_rect_size);
        
        task.left_rectified = left_resized.clone();
        task.right_rectified = right_resized.clone();
        task.submit_time = t0;
        task.rectify_time = t1;
        task.infer_time = t2;

        m_async_tasks.push_back(std::move(task));
        
        // LOG_INFO("Async task submitted. ");

        // 4. Limit queue length

    }

    /**
     * @brief Process completed async inference tasks
     */
    void ProcessCompletedTasks() {
        auto it = m_async_tasks.begin();
        while (it != m_async_tasks.end()) {
            // Non-blocking check for inference completion
            if (it->disparity_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                const auto t2 = std::chrono::high_resolution_clock::now();

                // Get disparity result
                cv::Mat disparity_map;
                try {
                    // LOG_INFO("Before  future.get(), async_tasks size: %d", m_async_tasks.size());
                    disparity_map = it->disparity_future.get().clone();  // Deep copy to avoid memory issues
                    // LOG_INFO("After future.get() and clone, async_tasks size: %d" , m_async_tasks.size());
                    
                } catch (const std::exception& e) {
                    LOG_ERROR("Exception while getting async result: %s", e.what());
                    it = m_async_tasks.erase(it);
                    continue;
                }

                if (disparity_map.empty()) {
                    LOG_WARN("Got empty disparity from async inference!");
                    it = m_async_tasks.erase(it);
                    continue;
                }

                // LOG_INFO("Disparity map size: %d, %d", disparity_map.rows, disparity_map.cols);
                // Copy required data before erase (avoid iterator invalidation)
                LidarData lidar_data_copy = it->lidar_data;
                StereoImage stereo_image_copy = it->stereo_image;  // Already deep-copied in SubmitAsyncTask, direct assignment is safe here
                cv::Mat left_rect_copy = it->left_rectified.clone();
                auto submit_time_copy = it->submit_time;
                auto rectify_time_copy = it->rectify_time;
                auto infer_time_copy = it->infer_time;

                // Build result object
                auto results = std::make_shared<DepthInferenceResults>();
                results->lidar_data = lidar_data_copy;
                results->stereo_image = stereo_image_copy;

                DepthImage& depth_img = results->depth_image;
                depth_img.img_width = m_infer_img_width;
                depth_img.img_height = m_infer_img_height;
                depth_img.bits_size = 32;
                depth_img.timestamp = it->stereo_image.timestamp;

                size_t depth_buffer_size = depth_img.img_width * depth_img.img_height * (depth_img.bits_size / 8);
                depth_img.depth_buffer = std::make_shared<std::vector<uint8_t>>(depth_buffer_size);
            

                // Size validation
                size_t expected_size = depth_img.img_width * depth_img.img_height * (depth_img.bits_size / 8);
                
                size_t actual_size = disparity_map.total() * disparity_map.elemSize();
                // LOG_INFO("Disparity map size1: %d, %d", disparity_map.rows, disparity_map.cols);

                if (actual_size == expected_size) {
                    memcpy(depth_img.depth_buffer->data(), disparity_map.data, actual_size);
                    
                    // Generate colored point cloud (using copied data)
                    GenerateColorPointCloud(disparity_map, left_rect_copy, results);
                } else {
                    LOG_ERROR("Disparity map size mismatch. Expected %d, got %d", expected_size, actual_size);
                }

                // LOG_INFO("Pointcloud generating success, point size:  %d", results->color_point_cloud.point_num);
                const auto t3 = std::chrono::high_resolution_clock::now();

                // Remove processed task from queue
                it = m_async_tasks.erase(it);

                // Publish result through callback
                if (m_callback) {
                    m_callback(results);
                }
            } else {
                // Not ready yet, check next one
                ++it;
            }
        }
    }

    /**
     * @brief Generate colored point cloud
     */
    void GenerateColorPointCloud(const cv::Mat& disparity_map, const cv::Mat& color_image,
                                  std::shared_ptr<DepthInferenceResults>& results) {
        // Get Q matrix (needs scaling according to inference size)
        cv::Mat Q = m_stereo_rectifier->GetQMatrix().clone();
        // float scale = static_cast<float>(m_infer_img_width) / image_data.img_width;
        // float  scale = m_image_scale;
        Q.at<double>(0, 3) *= m_image_scale;
        Q.at<double>(1, 3) *= m_image_scale;
        Q.at<double>(2, 3) *= m_image_scale;
        Q.at<double>(3, 3) *= m_image_scale;

        // 3D reprojection
        cv::Mat xyz;
        cv::reprojectImageTo3D(disparity_map, xyz, Q, true);

        // Generate point cloud
        std::vector<ColorPoint3D> point_cloud;
        point_cloud.reserve(xyz.rows * xyz.cols);

        for (int v = 0; v < xyz.rows; ++v) {
            for (int u = 0; u < xyz.cols; ++u) {
                cv::Vec3f point = xyz.at<cv::Vec3f>(v, u);
                
                // Filter invalid points
                if (std::isinf(point[2]) || std::isnan(point[2]) || 
                    point[2] > 100.0 || point[2] <= 0) {
                    continue;
                }

                ColorPoint3D p;
                p.x = point[0];
                p.y = point[1];
                p.z = point[2];

                // Extract color (BGR format)
                cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);

                p.r = color[0];
                p.g = color[1];
                p.b = color[2];

                p.a = 255;

                point_cloud.push_back(p);
            }
        }

        // Fill output result
        results->color_point_cloud.point_num = point_cloud.size();
        results->color_point_cloud.point_step = sizeof(ColorPoint3D);
        results->color_point_cloud.timestamp = results->depth_image.timestamp;
        
        // if (point_cloud.empty()) {
        //     results->color_point_cloud.point_cloud_buffer = std::make_shared<std::vector<uint8_t>>();
        //     return;
        // }
        
        size_t cloud_buffer_size = point_cloud.size() * sizeof(ColorPoint3D);
        results->color_point_cloud.point_cloud_buffer = 
            std::make_shared<std::vector<uint8_t>>(cloud_buffer_size);
        
        std::memcpy(results->color_point_cloud.point_cloud_buffer->data(),
                    point_cloud.data(), cloud_buffer_size);
    }
};  // class DepthInferenceImpl

// Factory function implementation
std::unique_ptr<DepthInferenceInterface> createDepthEstimator() {
    return std::make_unique<DepthInferenceImpl>();
}

} // namespace ac_depth
} // namespace robosense
