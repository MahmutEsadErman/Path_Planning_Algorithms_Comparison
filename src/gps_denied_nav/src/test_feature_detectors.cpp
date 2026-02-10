/**
 * @file test_feature_detectors.cpp
 * @brief Compare feature extraction methods (SIFT, SURF, ORB) across
 *        yaw calculation methods (Essential Matrix, Median Displacement, Hybrid EM+Avg)
 * 
 * Reads a rosbag directly and processes consecutive frames.
 * 
 * Usage: ros2 run gps_denied_nav test_feature_detectors --ros-args \
 *        -p bag_file_path:=/path/to/bag \
 *        -p max_pairs:=100 \
 *        -p skip_frames:=10
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <map>

#include "rclcpp/serialization.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

// ============== Data Structures ==============

struct FrameInfo {
    cv::Mat gray_image;
    double altitude;
    struct { double x, y, z; } position;
    struct { double x, y, z, w; } orientation;
};

struct MethodResult {
    std::string name;
    double yaw_rad;
    double time_us;
    bool valid;
};

struct DetectorResult {
    std::string detector_name;
    
    // Per-pair data
    struct PairResult {
        int pair_idx;
        int num_matches;
        double ground_truth_yaw;  // radians
        double feature_detect_time_us;  // feature detection time per frame
        MethodResult essential_matrix;
        MethodResult median_displacement;
        MethodResult hybrid_em_avg;
    };
    
    std::vector<PairResult> pairs;
    double total_detect_time_us;  // total feature detection time
    double avg_detect_time_us;    // average per-frame
};

// ============== Main Tester Class ==============

class FeatureDetectorTester : public rclcpp::Node {
public:
    FeatureDetectorTester() : Node("test_feature_detectors") {
        // Parameters
        this->declare_parameter<std::string>("bag_file_path", "yeni_harita");
        this->declare_parameter<double>("camera_pitch_angle", 90.0);
        this->declare_parameter<int>("iterations", 10);
        this->declare_parameter<int>("max_pairs", 50);
        this->declare_parameter<int>("skip_frames", 10);
        this->declare_parameter<double>("min_altitude", 49.0);
        this->declare_parameter<double>("starting_second", 1.0);
        this->declare_parameter<int>("similarity_threshold", 150);
        
        bag_file_path_ = this->get_parameter("bag_file_path").as_string();
        camera_pitch_angle_ = this->get_parameter("camera_pitch_angle").as_double();
        iterations_ = this->get_parameter("iterations").as_int();
        max_pairs_ = this->get_parameter("max_pairs").as_int();
        skip_frames_ = this->get_parameter("skip_frames").as_int();
        min_altitude_ = this->get_parameter("min_altitude").as_double();
        starting_second_ = this->get_parameter("starting_second").as_double();
        similarity_threshold_ = this->get_parameter("similarity_threshold").as_int();
        
        // Initialize camera transform
        initCameraTransform();
        
        // Read frames from bag
        RCLCPP_INFO(this->get_logger(), "Reading frames from bag: %s", bag_file_path_.c_str());
        readBag();
        
        if (frames_.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Not enough frames extracted from bag (%zu)", frames_.size());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Extracted %zu frames from bag", frames_.size());
        
        // Run tests for each detector
        std::vector<std::string> detectors = {"SIFT", "SURF", "ORB"};
        for (const auto& det : detectors) {
            RCLCPP_INFO(this->get_logger(), "Testing detector: %s", det.c_str());
            auto result = testDetector(det);
            results_.push_back(result);
        }
        
        // Print results
        printResults();
    }

private:
    std::string bag_file_path_;
    double camera_pitch_angle_;
    int iterations_;
    int max_pairs_;
    int skip_frames_;
    double min_altitude_;
    double starting_second_;
    int similarity_threshold_;
    
    cv::Mat K_;
    cv::Mat cam_tf_;
    
    std::vector<FrameInfo> frames_;
    std::vector<DetectorResult> results_;
    
    // ============== Initialization ==============
    
    void initCameraTransform() {
        cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,
            -1,  0,  0,
             0, -1,  0);

        double angle_rad = camera_pitch_angle_ * M_PI / 180.0;
        cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
            cos(angle_rad), 0, sin(angle_rad),
                         0, 1,              0,
           -sin(angle_rad), 0, cos(angle_rad));

        cam_tf_ = C_B_Cros * C_Cros_Ccv;
    }
    
    // ============== Bag Reading ==============
    
    template<typename T>
    std::shared_ptr<T> deserializeMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_message = std::make_shared<T>();
        rclcpp::Serialization<T> serialization;
        serialization.deserialize_message(&serialized_msg, ros_message.get());
        return ros_message;
    }
    
    void readBag() {
        auto reader = std::make_shared<rosbag2_cpp::Reader>();
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_path_;
        reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader->open(storage_options);
        
        bool K_received = false;
        double last_valid_altitude = 0.0;
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> last_pose_msg = nullptr;
        geometry_msgs::msg::Quaternion last_orientation;
        last_orientation.w = 1.0;
        int64_t first_timestamp = -1;
        
        while (reader->has_next()) {
            auto msg = reader->read_next();
            
            if (msg->topic_name == "/camera/camera_info" && !K_received) {
                auto camera_info_msg = deserializeMessage<sensor_msgs::msg::CameraInfo>(msg);
                K_ = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; i++) {
                    K_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
                }
                K_received = true;
            }
            else if (msg->topic_name == "/mavros/global_position/rel_alt") {
                auto alt_msg = deserializeMessage<std_msgs::msg::Float64>(msg);
                last_valid_altitude = alt_msg->data;
            }
            else if (msg->topic_name == "/simulation_pose_info") {
                last_pose_msg = msg;
            }
            else if (msg->topic_name == "/mavros/imu/data") {
                auto imu_msg = deserializeMessage<sensor_msgs::msg::Imu>(msg);
                last_orientation = imu_msg->orientation;
            }
            else if (msg->topic_name == "/camera/image") {
                auto image_msg = deserializeMessage<sensor_msgs::msg::Image>(msg);
                
                int64_t current_timestamp = rclcpp::Time(image_msg->header.stamp).nanoseconds();
                if (first_timestamp < 0) {
                    first_timestamp = current_timestamp;
                }
                
                double relative_time = (current_timestamp - first_timestamp) / 1e9;
                if (relative_time < starting_second_) continue;
                
                if (last_valid_altitude < min_altitude_) continue;
                
                auto cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
                cv::Mat gray;
                cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
                
                FrameInfo frame;
                frame.gray_image = gray;
                frame.altitude = last_valid_altitude;
                
                if (last_pose_msg) {
                    auto pose_msg = deserializeMessage<geometry_msgs::msg::PoseArray>(last_pose_msg);
                    if (pose_msg->poses.size() > 2) {
                        frame.position.x = pose_msg->poses[2].position.x;
                        frame.position.y = pose_msg->poses[2].position.y;
                        frame.position.z = pose_msg->poses[2].position.z;
                        frame.orientation.x = pose_msg->poses[2].orientation.x;
                        frame.orientation.y = pose_msg->poses[2].orientation.y;
                        frame.orientation.z = pose_msg->poses[2].orientation.z;
                        frame.orientation.w = pose_msg->poses[2].orientation.w;
                    }
                }
                
                frames_.push_back(frame);
            }
        }
    }
    
    // ============== Ground Truth ==============
    
    double quaternionToYaw(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    double calculateGroundTruthYaw(const FrameInfo& frame1, const FrameInfo& frame2) {
        double drone_yaw = quaternionToYaw(
            frame1.orientation.x, frame1.orientation.y,
            frame1.orientation.z, frame1.orientation.w);
        
        double dx_global = frame2.position.x - frame1.position.x;
        double dy_global = frame2.position.y - frame1.position.y;
        
        double global_motion_yaw = std::atan2(dy_global, dx_global);
        double relative_yaw = global_motion_yaw - drone_yaw;
        
        while (relative_yaw > M_PI) relative_yaw -= 2 * M_PI;
        while (relative_yaw < -M_PI) relative_yaw += 2 * M_PI;
        
        return relative_yaw;
    }
    
    // ============== Feature Detection ==============
    
    struct DetectedFeatures {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        double detect_time_us;
    };
    
    DetectedFeatures detectFeatures(const cv::Mat& gray, cv::Ptr<cv::Feature2D>& detector) {
        DetectedFeatures result;
        
        auto start = std::chrono::high_resolution_clock::now();
        detector->detectAndCompute(gray, cv::noArray(), result.keypoints, result.descriptors);
        auto end = std::chrono::high_resolution_clock::now();
        
        result.detect_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count();
        
        return result;
    }
    
    // ============== Feature Matching ==============
    
    std::vector<cv::DMatch> matchFeatures(const cv::Mat& des1, const cv::Mat& des2,
                                           cv::Ptr<cv::DescriptorMatcher>& matcher) {
        std::vector<cv::DMatch> good_matches;
        if (des1.rows < 2 || des2.rows < 2) return good_matches;
        
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(des1, des2, matches, 2);
        
        for (const auto& m : matches) {
            if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
                good_matches.push_back(m[0]);
            }
        }
        return good_matches;
    }
    
    // ============== Yaw Methods ==============
    
    MethodResult essentialMatrixMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Essential Matrix";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.size() < 10) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            cv::Mat E, R, t, mask;
            E = cv::findEssentialMat(pts1, pts2, K_, cv::USAC_MAGSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, pts1, pts2, K_, R, t, mask);
            
            cv::Mat t_body = cam_tf_ * (-t);
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    MethodResult medianDisplacementMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Median Displacement";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.empty()) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            std::vector<double> dx_vals, dy_vals;
            dx_vals.reserve(pts1_norm.size());
            dy_vals.reserve(pts1_norm.size());
            
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                dx_vals.push_back(pts2_norm[j].x - pts1_norm[j].x);
                dy_vals.push_back(pts2_norm[j].y - pts1_norm[j].y);
            }
            
            std::sort(dx_vals.begin(), dx_vals.end());
            std::sort(dy_vals.begin(), dy_vals.end());
            
            double median_dx = dx_vals[dx_vals.size() / 2];
            double median_dy = dy_vals[dy_vals.size() / 2];
            
            cv::Mat t_cam = (cv::Mat_<double>(3, 1) << -median_dx, -median_dy, 1.0);
            cv::Mat t_body = cam_tf_ * t_cam;
            
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    MethodResult hybridMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Hybrid EM+Avg";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.size() < 10) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            // Fast Avg Displacement as reference
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            double sum_dx = 0.0, sum_dy = 0.0;
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                sum_dx += pts2_norm[j].x - pts1_norm[j].x;
                sum_dy += pts2_norm[j].y - pts1_norm[j].y;
            }
            double avg_dx = sum_dx / pts1_norm.size();
            double avg_dy = sum_dy / pts1_norm.size();
            
            cv::Mat t_cam_avg = (cv::Mat_<double>(3, 1) << -avg_dx, -avg_dy, 1.0);
            cv::Mat t_body_avg = cam_tf_ * t_cam_avg;
            double avg_yaw = std::atan2(t_body_avg.at<double>(1), t_body_avg.at<double>(0));
            
            // Essential Matrix
            cv::Mat E, R, t, mask;
            E = cv::findEssentialMat(pts1, pts2, K_, cv::USAC_MAGSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, pts1, pts2, K_, R, t, mask);
            
            cv::Mat t_body_em = cam_tf_ * (-t);
            double em_yaw = std::atan2(t_body_em.at<double>(1), t_body_em.at<double>(0));
            
            // Fusion logic
            double diff = std::abs(em_yaw - avg_yaw);
            while (diff > M_PI) diff -= 2 * M_PI;
            diff = std::abs(diff);
            
            if (diff < 0.52) {
                result.yaw_rad = em_yaw;
            } else if (diff > 2.62) {
                result.yaw_rad = em_yaw + M_PI;
                while (result.yaw_rad > M_PI) result.yaw_rad -= 2 * M_PI;
                while (result.yaw_rad < -M_PI) result.yaw_rad += 2 * M_PI;
            } else {
                result.yaw_rad = avg_yaw;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ============== Per-Detector Test ==============
    
    DetectorResult testDetector(const std::string& detector_name) {
        DetectorResult det_result;
        det_result.detector_name = detector_name;
        det_result.total_detect_time_us = 0.0;
        
        // Create detector and matcher
        cv::Ptr<cv::Feature2D> detector;
        cv::Ptr<cv::DescriptorMatcher> matcher;
        
        if (detector_name == "ORB") {
            detector = cv::ORB::create();
            matcher = cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        } else if (detector_name == "SURF") {
            detector = cv::xfeatures2d::SURF::create(400);
            matcher = cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::KDTreeIndexParams>(5));
        } else {
            detector = cv::SIFT::create();
            matcher = cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::KDTreeIndexParams>(5));
        }
        
        // Pre-detect features for all frames
        struct FrameFeatures {
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            double detect_time_us;
        };
        
        std::vector<FrameFeatures> all_features(frames_.size());
        
        for (size_t i = 0; i < frames_.size(); ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            detector->detectAndCompute(frames_[i].gray_image, cv::noArray(),
                                       all_features[i].keypoints,
                                       all_features[i].descriptors);
            auto end = std::chrono::high_resolution_clock::now();
            all_features[i].detect_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start).count();
            det_result.total_detect_time_us += all_features[i].detect_time_us;
        }
        
        det_result.avg_detect_time_us = det_result.total_detect_time_us / frames_.size();
        
        // Test consecutive frame pairs
        int pairs_tested = 0;
        
        // Use similarity-based frame selection (like create_path does)
        // First, find which frames are "different enough" to form pairs
        std::vector<size_t> selected_indices;
        selected_indices.push_back(0);
        
        for (size_t i = 1; i < frames_.size(); ++i) {
            if (all_features[i].descriptors.empty() || 
                all_features[selected_indices.back()].descriptors.empty()) continue;
            
            // Check if this frame is sufficiently different from last selected
            auto matches = matchFeatures(
                all_features[selected_indices.back()].descriptors,
                all_features[i].descriptors, matcher);
            
            if (matches.size() < static_cast<size_t>(similarity_threshold_)) {
                selected_indices.push_back(i);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "  %s: %zu selected frames from %zu total",
                     detector_name.c_str(), selected_indices.size(), frames_.size());
        
        for (size_t si = skip_frames_; 
             si < selected_indices.size() - 1 && pairs_tested < max_pairs_; ++si) {
            
            size_t idx1 = selected_indices[si];
            size_t idx2 = selected_indices[si + 1];
            
            auto& feat1 = all_features[idx1];
            auto& feat2 = all_features[idx2];
            
            if (feat1.descriptors.empty() || feat2.descriptors.empty()) continue;
            
            auto matches = matchFeatures(feat1.descriptors, feat2.descriptors, matcher);
            if (matches.size() < 10) continue;
            
            DetectorResult::PairResult pair;
            pair.pair_idx = si;
            pair.num_matches = matches.size();
            pair.feature_detect_time_us = (feat1.detect_time_us + feat2.detect_time_us) / 2.0;
            
            pair.ground_truth_yaw = calculateGroundTruthYaw(frames_[idx1], frames_[idx2]);
            
            pair.essential_matrix = essentialMatrixMethod(
                feat1.keypoints, feat2.keypoints, matches);
            pair.median_displacement = medianDisplacementMethod(
                feat1.keypoints, feat2.keypoints, matches);
            pair.hybrid_em_avg = hybridMethod(
                feat1.keypoints, feat2.keypoints, matches);
            
            det_result.pairs.push_back(pair);
            pairs_tested++;
        }
        
        RCLCPP_INFO(this->get_logger(), "  %s: tested %d pairs", 
                     detector_name.c_str(), pairs_tested);
        
        return det_result;
    }
    
    // ============== Results Printing ==============
    
    void printResults() {
        if (results_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No results to display!");
            return;
        }
        
        // Compute stats per detector per method
        struct Stats {
            std::vector<double> errors;
            std::vector<double> times;
            double mean_error = 0;
            double std_error = 0;
            double max_error = 0;
            double mean_time = 0;
        };
        
        struct DetectorStats {
            std::string name;
            int num_pairs;
            int avg_matches;
            double avg_detect_time_us;
            Stats em, med, hybrid;
        };
        
        std::vector<DetectorStats> all_stats;
        
        for (const auto& det : results_) {
            DetectorStats ds;
            ds.name = det.detector_name;
            ds.num_pairs = det.pairs.size();
            ds.avg_detect_time_us = det.avg_detect_time_us;
            
            int total_matches = 0;
            
            for (const auto& p : det.pairs) {
                double gt = p.ground_truth_yaw * 180.0 / M_PI;
                total_matches += p.num_matches;
                
                auto addStats = [&](const MethodResult& mr, Stats& s) {
                    if (mr.valid) {
                        double err = std::abs(mr.yaw_rad * 180.0 / M_PI - gt);
                        if (err > 180) err = 360 - err;
                        s.errors.push_back(err);
                        s.times.push_back(mr.time_us);
                    }
                };
                
                addStats(p.essential_matrix, ds.em);
                addStats(p.median_displacement, ds.med);
                addStats(p.hybrid_em_avg, ds.hybrid);
            }
            
            ds.avg_matches = ds.num_pairs > 0 ? total_matches / ds.num_pairs : 0;
            
            auto calcStats = [](Stats& s) {
                if (s.errors.empty()) return;
                s.mean_error = std::accumulate(s.errors.begin(), s.errors.end(), 0.0) / s.errors.size();
                s.max_error = *std::max_element(s.errors.begin(), s.errors.end());
                s.mean_time = std::accumulate(s.times.begin(), s.times.end(), 0.0) / s.times.size();
                double sq_sum = 0;
                for (double e : s.errors) sq_sum += (e - s.mean_error) * (e - s.mean_error);
                s.std_error = std::sqrt(sq_sum / s.errors.size());
            };
            
            calcStats(ds.em);
            calcStats(ds.med);
            calcStats(ds.hybrid);
            
            all_stats.push_back(ds);
        }
        
        // ============== Print Header ==============
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                     FEATURE DETECTOR COMPARISON                                        ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  Bag File: " << std::setw(76) << std::left << bag_file_path_ << "║\n";
        std::cout << "║  Total Frames: " << std::setw(72) << std::left << frames_.size() << "║\n";
        std::cout << "║  Iterations per Method: " << std::setw(63) << std::left << iterations_ << "║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════════════════════════════╝\n";
        
        std::cout << std::fixed << std::setprecision(2);
        
        // ============== Per-Detector Summary ==============
        for (const auto& ds : all_stats) {
            std::cout << "\n";
            std::cout << "┌──────────────────────────────────────────────────────────────────────────────────────────┐\n";
            std::cout << "│  Detector: " << std::setw(76) << std::left << ds.name << "│\n";
            std::cout << "│  Pairs Tested: " << std::setw(72) << std::left << ds.num_pairs << "│\n";
            std::cout << "│  Avg Matches: " << std::setw(73) << std::left << ds.avg_matches << "│\n";
            std::cout << "│  Avg Feature Detection Time: " 
                      << std::setw(55) << std::left 
                      << (std::to_string((int)ds.avg_detect_time_us) + " µs") << "  │\n";
            std::cout << "├────────────────────────────┬───────────────┬───────────────┬───────────────┬───────────┤\n";
            std::cout << "│         Method             │   Mean Error  │   Std Dev     │   Max Error   │ Time(µs)  │\n";
            std::cout << "├────────────────────────────┼───────────────┼───────────────┼───────────────┼───────────┤\n";
            
            auto printRow = [](const std::string& name, const Stats& s) {
                std::cout << "│  " << std::setw(25) << std::left << name
                          << " │     " << std::setw(8) << s.mean_error
                          << "  │     " << std::setw(8) << s.std_error
                          << "  │     " << std::setw(8) << s.max_error
                          << "  │ " << std::setw(8) << s.mean_time << "│\n";
            };
            
            printRow("Essential Matrix", ds.em);
            printRow("Median Displacement", ds.med);
            printRow("Hybrid EM+Avg", ds.hybrid);
            
            std::cout << "└────────────────────────────┴───────────────┴───────────────┴───────────────┴───────────┘\n";
        }
        
        // ============== Cross-Detector Comparison Table ==============
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                        CROSS-DETECTOR COMPARISON (Mean Error °)                        ║\n";
        std::cout << "╠════════════════╦═══════════════════╦═══════════════════════╦════════════════════════════╣\n";
        std::cout << "║    Detector    ║  Essential Matrix ║  Median Displacement  ║      Hybrid EM+Avg        ║\n";
        std::cout << "╠════════════════╬═══════════════════╬═══════════════════════╬════════════════════════════╣\n";
        
        for (const auto& ds : all_stats) {
            std::cout << "║  " << std::setw(13) << std::left << ds.name
                      << " ║     " << std::setw(12) << ds.em.mean_error
                      << "  ║     " << std::setw(16) << ds.med.mean_error
                      << "  ║     " << std::setw(21) << ds.hybrid.mean_error << "  ║\n";
        }
        
        std::cout << "╠════════════════╩═══════════════════╩═══════════════════════╩════════════════════════════╣\n";
        std::cout << "║                        CROSS-DETECTOR COMPARISON (Speed)                               ║\n";
        std::cout << "╠════════════════╦═══════════════════╦═══════════════════════╦════════════════════════════╣\n";
        std::cout << "║    Detector    ║  Detect Time(µs)  ║  EM Yaw Time(µs)      ║  Total per Frame(µs)      ║\n";
        std::cout << "╠════════════════╬═══════════════════╬═══════════════════════╬════════════════════════════╣\n";
        
        for (const auto& ds : all_stats) {
            double total_time = ds.avg_detect_time_us + ds.em.mean_time;
            std::cout << "║  " << std::setw(13) << std::left << ds.name
                      << " ║     " << std::setw(12) << ds.avg_detect_time_us
                      << "  ║     " << std::setw(16) << ds.em.mean_time
                      << "  ║     " << std::setw(21) << total_time << "  ║\n";
        }
        
        std::cout << "╠════════════════╩═══════════════════╩═══════════════════════╩════════════════════════════╣\n";
        
        // ============== Recommendation ==============
        // Find best detector+method combination
        double best_error = 999;
        std::string best_combo;
        
        for (const auto& ds : all_stats) {
            struct { std::string method; double err; } methods[] = {
                {"Essential Matrix", ds.em.mean_error},
                {"Median Displacement", ds.med.mean_error},
                {"Hybrid EM+Avg", ds.hybrid.mean_error}
            };
            for (const auto& m : methods) {
                if (m.err < best_error && m.err > 0) {
                    best_error = m.err;
                    best_combo = ds.name + " + " + m.method;
                }
            }
        }
        
        std::string recommendation = "BEST: " + best_combo + " (" + 
            std::to_string(best_error).substr(0, 5) + "° mean error)";
        
        std::cout << "║                                 RECOMMENDATION                                        ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  " << std::setw(87) << std::left << recommendation << "║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════════════════════════════╝\n";
        
        // ============== Sample Comparisons per Detector ==============
        for (const auto& det : results_) {
            std::cout << "\nSample Pairs - " << det.detector_name << " (first 10):\n";
            std::cout << "────────────────────────────────────────────────────────────────────────────────────────\n";
            std::cout << std::setw(6) << "Pair"
                      << std::setw(8) << "Match"
                      << std::setw(10) << "GT"
                      << std::setw(10) << "EM"
                      << std::setw(10) << "Median"
                      << std::setw(10) << "Hybrid"
                      << std::setw(10) << "EM_err"
                      << std::setw(10) << "Med_err"
                      << std::setw(10) << "Hyb_err" << "\n";
            std::cout << "────────────────────────────────────────────────────────────────────────────────────────\n";
            
            for (size_t i = 0; i < std::min(size_t(10), det.pairs.size()); ++i) {
                const auto& p = det.pairs[i];
                double gt = p.ground_truth_yaw * 180.0 / M_PI;
                double em = p.essential_matrix.yaw_rad * 180.0 / M_PI;
                double med = p.median_displacement.yaw_rad * 180.0 / M_PI;
                double hyb = p.hybrid_em_avg.yaw_rad * 180.0 / M_PI;
                
                auto calcError = [](double val, double gt) {
                    double err = std::abs(val - gt);
                    if (err > 180) err = 360 - err;
                    return err;
                };
                
                std::cout << std::setw(6) << p.pair_idx
                          << std::setw(8) << p.num_matches
                          << std::setw(10) << gt
                          << std::setw(10) << em
                          << std::setw(10) << med
                          << std::setw(10) << hyb
                          << std::setw(10) << calcError(em, gt)
                          << std::setw(10) << calcError(med, gt)
                          << std::setw(10) << calcError(hyb, gt) << "\n";
            }
        }
        std::cout << "\n";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureDetectorTester>();
    rclcpp::shutdown();
    return 0;
}
