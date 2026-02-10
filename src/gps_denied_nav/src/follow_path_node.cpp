/**
 * @file follow_path_node.cpp
 * @brief Implementation of the FollowPathNode ROS2 node
 */

#include "gps_denied_nav/follow_path_node.hpp"
#include <cmath>
#include <algorithm>

namespace gps_denied_nav {

FollowPathNode::FollowPathNode() : Node("follow_path_node")
{
    // Initialize state
    path_index_ = 0;
    ready_ = false;
    lost_path_ = false;
    returning_ = false;
    forward_search_attempts_ = 0;
    max_forward_search_range_ = 15;

    // Declare parameters
    this->declare_parameter<std::string>("path_file", "yeni_harita_SURF.yaml");
    this->declare_parameter<double>("camera_pitch_angle", 90.0);
    this->declare_parameter<int>("similarity_threshold", 100);
    this->declare_parameter<int>("min_feature_count", 25);
    this->declare_parameter<double>("yaw_kp", 0.05);
    this->declare_parameter<double>("velocity", 5.0);
    this->declare_parameter<bool>("debug", true);

    // Get parameters
    similarity_threshold_ = this->get_parameter("similarity_threshold").as_int();
    min_feature_count_ = this->get_parameter("min_feature_count").as_int();
    vel_ = this->get_parameter("velocity").as_double();
    camera_pitch_angle_ = this->get_parameter("camera_pitch_angle").as_double();
    path_file_ = this->get_parameter("path_file").as_string();
    yaw_kp_ = this->get_parameter("yaw_kp").as_double();
    DEBUG_ = this->get_parameter("debug").as_bool();

    // Create marker publisher
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/last_point_marker", 10);

    // Load path from file
    loadPath(path_file_);

    // Calculate camera transform matrix
    // Step 1: Define C_Cros_Ccv (OpenCV Cam to ROS-style Cam)
    cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
         0,  0,  1,   // ROS X = CV Z
        -1,  0,  0,   // ROS Y = -CV X
         0, -1,  0);  // ROS Z = -CV Y

    // Step 2: Define C_B_Cros (ROS-style Cam to Drone Body)
    double angle_rad = camera_pitch_angle_ * M_PI / 180.0;
    cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
        cos(angle_rad), 0, sin(angle_rad),
                     0, 1,              0,
       -sin(angle_rad), 0, cos(angle_rad));

    // Step 3: Combine them to get C_B_Ccv (OpenCV Cam to Drone Body)
    cam_tf_ = C_B_Cros * C_Cros_Ccv;

    // Initialize feature processor
    feature_processor_ = std::make_unique<FeatureProcessor>(feature_detector_, K_, cam_tf_);

    // Create subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image",
        rclcpp::SensorDataQoS(),
        std::bind(&FollowPathNode::imageCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&FollowPathNode::imuCallback, this, std::placeholders::_1));

    gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/simulation_pose_info",
        rclcpp::SensorDataQoS(),
        std::bind(&FollowPathNode::gtPoseCallback, this, std::placeholders::_1));

    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local",
        rclcpp::SensorDataQoS(),
        std::bind(&FollowPathNode::velCallback, this, std::placeholders::_1));

    // Create publishers
    target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/target_pose", 10);

    returning_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/returning_status", 10);

    vel_publisher_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
}

FollowPathNode::~FollowPathNode()
{
    // Print all statistics on program exit
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - path_following_time_).count();

    double avg_error = (error_count_ > 0) ? (error_sum_ / error_count_) : 0.0;

    // Calculate distance to endpoint (if gt_pose_ is available)
    if (gt_pose_ && distance_to_endpoint_ < 0) {
        geometry_msgs::msg::Point target_point;
        if (returning_) {
            target_point = starting_position_;
        } else if (!path_data_.empty()) {
            target_point = path_data_.back().target_pose.position;
        }
        double dx = gt_pose_->position.x - target_point.x;
        double dy = gt_pose_->position.y - target_point.y;
        double dz = gt_pose_->position.z - target_point.z;
        distance_to_endpoint_ = std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    RCLCPP_INFO(this->get_logger(), "\n============================================================");
    RCLCPP_INFO(this->get_logger(), "                         STATISTICS                         ");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "[PARAMETERS]");
    RCLCPP_INFO(this->get_logger(), "  Path File:           %s", path_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Feature Detector:    %s", feature_detector_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Velocity:            %.2f m/s", vel_);
    RCLCPP_INFO(this->get_logger(), "  Similarity Threshold:%d", similarity_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Min Feature Count:   %d", min_feature_count_);
    RCLCPP_INFO(this->get_logger(), "  Camera Pitch Angle:  %.1f degrees", camera_pitch_angle_);
    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[PATH FOLLOWING]");
    RCLCPP_INFO(this->get_logger(), "  Path Index:          %d / %zu", path_index_+1, 
                lost_path_ ? path_history_.size() : path_data_.size());
    RCLCPP_INFO(this->get_logger(), "  Status:              %s",
        path_completed_ ? "COMPLETED" :
        (returning_ ? "RETURNING" : (lost_path_ ? "PATH LOST" : "FORWARD")));
    RCLCPP_INFO(this->get_logger(), "  Path History Size:   %zu", path_history_.size());
    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[PERFORMANCE]");
    RCLCPP_INFO(this->get_logger(), "  Total Time:          %ld seconds", total_time);
    RCLCPP_INFO(this->get_logger(), "  Frames Processed:    %d", frame_count_);
    RCLCPP_INFO(this->get_logger(), "  Average FPS:         %.2f", avg_fps_);
    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[ERROR ANALYSIS]");
    RCLCPP_INFO(this->get_logger(), "  Average Error:       %.4f meters", avg_error);
    RCLCPP_INFO(this->get_logger(), "  Total Measurements:  %d", error_count_);
    RCLCPP_INFO(this->get_logger(), "  Distance to Target:  %.4f meters", distance_to_endpoint_);
    RCLCPP_INFO(this->get_logger(), "============================================================\n");
}

// ========== Velocity Publishing Functions ==========

void FollowPathNode::velPublish(geometry_msgs::msg::Twist vel, double yaw)
{
    msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    msg_.type_mask =
          mavros_msgs::msg::PositionTarget::IGNORE_PX
        | mavros_msgs::msg::PositionTarget::IGNORE_PY
        | mavros_msgs::msg::PositionTarget::IGNORE_PZ
        | mavros_msgs::msg::PositionTarget::IGNORE_AFX
        | mavros_msgs::msg::PositionTarget::IGNORE_AFY
        | mavros_msgs::msg::PositionTarget::IGNORE_AFZ
        | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    msg_.velocity = vel.linear;
    msg_.yaw = yaw;

    vel_publisher_->publish(msg_);
}

void FollowPathNode::yawPublish(double yaw)
{
    msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    msg_.type_mask =
          mavros_msgs::msg::PositionTarget::IGNORE_PX
        | mavros_msgs::msg::PositionTarget::IGNORE_PY
        | mavros_msgs::msg::PositionTarget::IGNORE_PZ
        | mavros_msgs::msg::PositionTarget::IGNORE_AFX
        | mavros_msgs::msg::PositionTarget::IGNORE_AFY
        | mavros_msgs::msg::PositionTarget::IGNORE_AFZ
        | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    msg_.velocity.x = 0.0;
    msg_.velocity.y = 0.0;
    msg_.velocity.z = 0.0;
    msg_.yaw = yaw;

    vel_publisher_->publish(msg_);
}

// ========== Alignment and Path Following ==========

void FollowPathNode::alignToTarget(const cv::Mat& target_descriptors,
                                    const std::vector<cv::KeyPoint>& target_keypoints,
                                    const cv::Mat& current_descriptors,
                                    const std::vector<cv::KeyPoint>& current_keypoints,
                                    bool is_initial_alignment)
{
    std::vector<cv::DMatch> matches;
    feature_processor_->compareFeatures(target_descriptors, current_descriptors, matches);

    double target_yaw = drone_yaw_;
    double relative_yaw = 0.0;

    if (matches.size() >= static_cast<size_t>(min_feature_count_)) {
        double raw_relative_yaw = feature_processor_->calculateRelativeRotation(
            target_keypoints, current_keypoints, matches);

        relative_yaw = filterYaw(raw_relative_yaw);
        target_yaw = drone_yaw_ + relative_yaw*yaw_kp_;

        if (DEBUG_) {
            RCLCPP_INFO(this->get_logger(), "Aligning - Raw: %.2f rad, Filtered: %.2f rad (%.1f deg), Drone yaw: %.2f",
                        raw_relative_yaw, relative_yaw, relative_yaw * 180.0 / M_PI, drone_yaw_);
        }
    } else {
        if (DEBUG_) {
            RCLCPP_WARN(this->get_logger(), "Not enough matches for alignment: %zu < %d",
                        matches.size(), min_feature_count_);
        }
    }

    yawPublish(target_yaw);

    // Check if aligned
    if (std::abs(relative_yaw) < 0.05 && current_vel_magnitude_ < 0.5) {
        if (is_initial_alignment) {
            starting_position_ = gt_pose_->position;
            publishLastPointMarker();
            path_following_time_ = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "Initial alignment complete! Relative yaw: %.2f rad (%.1f deg)",
                        relative_yaw, relative_yaw * 180.0 / M_PI);
        } else {
            RCLCPP_INFO(this->get_logger(), "Aligned to new target point. Relative yaw: %.2f rad", relative_yaw);
        }
        ready_ = true;
        clearYawFilter();
    }
}

void FollowPathNode::followPath(double vel, double target_yaw)
{
    double vel_x = vel;
    double vel_y = 0;
    double vel_z = 0;

    if (DEBUG_) {
        RCLCPP_INFO(this->get_logger(),
                    "yaw=%.2f rad, target_yaw=%.2f rad",
                    drone_yaw_, target_yaw);
    }

    target_yaw = yaw_kp_ * target_yaw + drone_yaw_;

    // Rotate body-frame velocity to global ENU
    double cos_yaw = std::cos(drone_yaw_);
    double sin_yaw = std::sin(drone_yaw_);

    double global_vx = cos_yaw * vel_x - sin_yaw * vel_y;
    double global_vy = sin_yaw * vel_x + cos_yaw * vel_y;
    double global_vz = vel_z;

    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = global_vx;
    vel_msg.linear.y = global_vy;
    vel_msg.linear.z = global_vz;
    velPublish(vel_msg, target_yaw);
}

int FollowPathNode::searchForwardPath(const cv::Mat& current_descriptors, int start_index)
{
    if (current_descriptors.empty()) {
        return -1;
    }

    int end_index = std::min(static_cast<int>(path_data_.size()), start_index + max_forward_search_range_);

    for (int i = start_index + 1; i < end_index; i++) {
        std::vector<cv::DMatch> matches;
        feature_processor_->compareFeatures(
            path_data_[i].features.descriptors, current_descriptors, matches);

        if (matches.size() >= static_cast<size_t>(min_feature_count_)) {
            RCLCPP_INFO(this->get_logger(),
                "Found matching path at index %d with %zu matches (skipped %d frames)",
                i, matches.size(), i - start_index);
            return i;
        }
    }

    RCLCPP_DEBUG(this->get_logger(),
        "No matching path found in range [%d, %d)", start_index + 1, end_index);
    return -1;
}

// ========== Yaw Filter ==========

double FollowPathNode::filterYaw(double raw_yaw)
{
    if (!filter_initialized_) {
        filtered_yaw_ = raw_yaw;
        filter_initialized_ = true;
        outlier_count_ = 0;
        if (DEBUG_) {
            RCLCPP_INFO(this->get_logger(), "Filter initialized with: %.2f rad", raw_yaw);
        }
        return filtered_yaw_;
    }

    // Use angular difference that properly handles ±π wraparound
    double angular_diff = std::atan2(std::sin(raw_yaw - filtered_yaw_),
                                     std::cos(raw_yaw - filtered_yaw_));
    double abs_diff = std::abs(angular_diff);

    if (abs_diff > 1.5) {  // ~86 degrees tolerance
        outlier_count_++;
        if (DEBUG_) {
            RCLCPP_WARN(this->get_logger(), "Outlier rejected: %.2f rad (angular_diff: %.2f, count: %d)",
                        raw_yaw, angular_diff, outlier_count_);
        }

        if (outlier_count_ > 10) {
            RCLCPP_WARN(this->get_logger(), "Too many outliers, resetting filter");
            filter_initialized_ = false;
        }

        return filtered_yaw_;
    }

    outlier_count_ = 0;

    // Circular Exponential Moving Average
    // Instead of linear interpolation, add weighted angular difference
    double alpha = 0.4;
    filtered_yaw_ = filtered_yaw_ + alpha * angular_diff;

    // Normalize to [-π, π]
    filtered_yaw_ = std::atan2(std::sin(filtered_yaw_), std::cos(filtered_yaw_));

    return filtered_yaw_;
}

void FollowPathNode::clearYawFilter()
{
    filter_initialized_ = false;
    filtered_yaw_ = 0.0;
    outlier_count_ = 0;
}

// ========== Utility Functions ==========

double FollowPathNode::calculateError()
{
    if (!error_calc_initialized_) {
        error_calc_m_ = std::tan(starting_yaw_);
        error_calc_payda_ = std::sqrt(error_calc_m_ * error_calc_m_ + 1);
        error_calc_initialized_ = true;
    }

    double x = gt_pose_->position.x - starting_position_.x;
    double y = gt_pose_->position.y - starting_position_.y;

    double error = std::abs(-error_calc_m_ * x + y) / error_calc_payda_;
    error_sum_ += error;
    error_count_++;
    return error_sum_;
}

void FollowPathNode::publishLastPointMarker()
{
    if (path_data_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot publish marker: path is empty");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "path_endpoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    const auto& last_pose = path_data_[path_data_.size() - 1].target_pose;
    marker.pose.position.x = last_pose.position.x - starting_position_.x;
    marker.pose.position.y = last_pose.position.y - starting_position_.y;
    marker.pose.position.z = starting_position_.z;

    marker.scale.x = 20;
    marker.scale.y = 20;
    marker.scale.z = 2;

    marker.color.a = 0.5;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.5;

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker_publisher_->publish(marker);
}

void FollowPathNode::loadPath(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for reading: %s", filename.c_str());
        return;
    }

    fs["K"] >> K_;
    fs["feature_detector"] >> feature_detector_;

    cv::FileNode frames = fs["frames"];
    if (frames.type() != cv::FileNode::SEQ) {
        RCLCPP_ERROR(this->get_logger(), "Invalid file format: 'frames' is not a sequence");
        return;
    }

    for (auto it = frames.begin(); it != frames.end(); ++it) {
        cv::FileNode frame_node = *it;
        FrameData frame;

        if ((double)frame_node["altitude"] < 49) {
            continue;
        }

        // Keypoints and Descriptors
        frame_node["keypoints"] >> frame.features.keypoints;
        frame_node["descriptors"] >> frame.features.descriptors;

        // IMU
        cv::FileNode imu_node = frame_node["imu"];
        cv::FileNode acc_node = imu_node["linear_acceleration"];
        frame.imu.linear_acceleration.x = (double)acc_node["x"];
        frame.imu.linear_acceleration.y = (double)acc_node["y"];
        frame.imu.linear_acceleration.z = (double)acc_node["z"];

        cv::FileNode gyro_node = imu_node["angular_velocity"];
        frame.imu.angular_velocity.x = (double)gyro_node["x"];
        frame.imu.angular_velocity.y = (double)gyro_node["y"];
        frame.imu.angular_velocity.z = (double)gyro_node["z"];

        cv::FileNode orient_node = imu_node["orientation"];
        frame.imu.orientation.x = (double)orient_node["x"];
        frame.imu.orientation.y = (double)orient_node["y"];
        frame.imu.orientation.z = (double)orient_node["z"];
        frame.imu.orientation.w = (double)orient_node["w"];

        // Altitude
        frame.altitude.data = (double)frame_node["altitude"];

        // Target Pose
        if (!frame_node["target_pose"].empty()) {
            cv::FileNode pos_node = frame_node["target_pose"]["position"];
            frame.target_pose.position.x = (double)pos_node["x"];
            frame.target_pose.position.y = (double)pos_node["y"];
            frame.target_pose.position.z = (double)pos_node["z"];
        }

        path_data_.push_back(frame);
    }

    fs.release();
    RCLCPP_INFO(this->get_logger(), "Loaded path with %zu frames", path_data_.size());
}

// ========== Callback Functions ==========

void FollowPathNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    drone_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void FollowPathNode::velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    double vz = msg->twist.linear.z;
    current_vel_magnitude_ = std::sqrt(vx*vx + vy*vy + vz*vz);
}

void FollowPathNode::gtPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (msg->poses.size() > 2) {
        gt_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[2]);
    } else {
        RCLCPP_WARN(this->get_logger(), "PoseArray does not contain enough poses.");
    }
}

void FollowPathNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // FPS calculation
    auto current_time = std::chrono::high_resolution_clock::now();
    if (!first_frame_) {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - last_frame_time_).count();
        fps_ = 1000000.0 / duration;
        fps_sum_ += fps_;
        frame_count_++;
        avg_fps_ = fps_sum_ / frame_count_;
    } else {
        first_frame_ = false;
    }
    last_frame_time_ = current_time;

    // For the first two paths we use a lower threshold to start easier
    double similarity_threshold = (path_index_ < 2) ? 60 : similarity_threshold_;
    
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
    std::vector<cv::DMatch> good_matches;
    double vel = vel_;
    double target_yaw = 0;

    if (!returning_) {
        // Detect features
        feature_processor_->detectAndCompute(gray_image, kp, des);

        // If not aligned yet
        if (!ready_) {
            alignToTarget(
                path_data_[path_index_].features.descriptors,
                path_data_[path_index_].features.keypoints,
                des, kp, true);
            return;
        }

        // Compare with last frame
        std::vector<cv::DMatch> last_frame_matches;
        if (!path_history_.empty()) {
            feature_processor_->compareFeatures(
                path_history_.back().first.descriptors, des, last_frame_matches);
        }

        // Save path history
        if (kp.size() >= static_cast<size_t>(similarity_threshold + 10) &&
            (path_history_.empty() || last_frame_matches.size() < static_cast<size_t>(similarity_threshold))) {
            Features frame;
            frame.keypoints = kp;
            frame.descriptors = des;
            path_history_.push_back(std::pair<Features, geometry_msgs::msg::Pose>(frame, *gt_pose_));
        }

        // If path is lost, search forward
        if (lost_path_) {
            int found_index = searchForwardPath(des, path_index_);

            if (found_index >= 0) {
                RCLCPP_INFO(this->get_logger(), "Forward search successful! Jumping from index %d to %d",
                            path_index_, found_index);
                path_index_ = found_index;
                lost_path_ = false;
                forward_search_attempts_ = 0;
                ready_ = false;
                return;
            } else {
                forward_search_attempts_++;
                RCLCPP_INFO(this->get_logger(), "Forward search attempt %d/%d failed",
                            forward_search_attempts_, 3);

                if (forward_search_attempts_ >= 3) {
                    yawPublish(drone_yaw_);
                    if (current_vel_magnitude_ < 0.5) {
                        path_index_ = 0;
                        returning_ = true;
                        ready_ = false;
                        forward_search_attempts_ = 0;

                        std_msgs::msg::Bool bool_msg;
                        bool_msg.data = true;
                        returning_publisher_->publish(bool_msg);

                        RCLCPP_INFO(this->get_logger(), "Forward search exhausted. Returning home.");
                        RCLCPP_INFO(this->get_logger(), "path history size: %ld", path_history_.size());
                    }
                    return;
                }
                followPath(1.0, 0);
                return;
            }
        }

        // Compare features with current path point
        feature_processor_->compareFeatures(
            path_data_[path_index_].features.descriptors, des, good_matches);

        if (good_matches.size() >= static_cast<size_t>(min_feature_count_)) {
            target_yaw = feature_processor_->calculateRelativeYaw(
                path_data_[path_index_].features.keypoints, kp, good_matches);
        } else if (match_size_buff_[9] != -1 && match_buff_sum_/10 < min_feature_count_) {
            lost_path_ = true;
            RCLCPP_INFO(this->get_logger(), "\n---Lost the path. Stopping...---\n");
        }

        if (good_matches.size() > static_cast<size_t>(similarity_threshold)) {
            if (path_index_ >= static_cast<int>(path_data_.size()) - 1) {
                vel = 0;

                if (gt_pose_ && !path_data_.empty()) {
                    const auto& target = path_data_.back().target_pose.position;
                    double dx = gt_pose_->position.x - target.x;
                    double dy = gt_pose_->position.y - target.y;
                    distance_to_endpoint_ = std::sqrt(dx*dx + dy*dy);
                }
                path_completed_ = true;

                double avg_error = calculateError() / error_count_;
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - path_following_time_).count();
                RCLCPP_INFO(this->get_logger(), "Path following completed. \n Average error: %f m", avg_error);
                RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                RCLCPP_INFO(this->get_logger(), "Distance to endpoint: %.4f m", distance_to_endpoint_);
                image_sub_.reset();
                return;
            }

            calculateError();
            path_index_ += 1;
            target_pose_publisher_->publish(path_data_[path_index_].target_pose);
        }
    } else {
        // Go back to home
        feature_processor_->detectAndCompute(gray_image, kp, des);

        if (!ready_) {
            int history_idx = path_history_.size() - 1 - path_index_;
            if (history_idx >= 0 && history_idx < static_cast<int>(path_history_.size())) {
                alignToTarget(
                    path_history_[history_idx].first.descriptors,
                    path_history_[history_idx].first.keypoints,
                    des, kp, path_index_ == 0);
            }
            return;
        }

        if (path_history_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path history is empty, cannot backtrack.");
            return;
        }

        int history_idx = path_history_.size() - 1 - path_index_;

        feature_processor_->compareFeatures(
            path_history_[history_idx].first.descriptors, des, good_matches);

        if (good_matches.size() >= static_cast<size_t>(min_feature_count_)) {
            target_yaw = feature_processor_->calculateRelativeYaw(
                path_history_[history_idx].first.keypoints, kp, good_matches);
        }

        if (good_matches.size() > static_cast<size_t>(similarity_threshold)) {
            if (path_index_ >= static_cast<int>(path_history_.size()) - 1) {
                vel = 0;

                if (gt_pose_) {
                    double dx = gt_pose_->position.x - starting_position_.x;
                    double dy = gt_pose_->position.y - starting_position_.y;
                    distance_to_endpoint_ = std::sqrt(dx*dx + dy*dy);
                }
                path_completed_ = true;

                image_sub_.reset();
                double avg_error = calculateError() / error_count_;
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - path_following_time_).count();
                RCLCPP_INFO(this->get_logger(), "Backtracking completed. \n Average error: %f m", avg_error);
                RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                RCLCPP_INFO(this->get_logger(), "Distance to start point: %.4f m", distance_to_endpoint_);
                return;
            }

            calculateError();
            path_index_ += 1;
        }
    }

    if (path_index_ > 1) {
        if ((returning_ && path_index_ >= static_cast<int>(path_history_.size()) - 2) ||
            (!returning_ && path_index_ >= static_cast<int>(path_data_.size()) - 2)) {
            vel = 2;
        }
        followPath(vel, target_yaw);
        
        if (!lost_path_) {
            match_buff_sum_ -= match_size_buff_[match_frame_index_];
            match_buff_sum_ += good_matches.size();
            match_size_buff_[match_frame_index_] = good_matches.size();
            match_frame_index_ = (match_frame_index_ + 1) % 10;
            
            if (DEBUG_) {
                RCLCPP_INFO(this->get_logger(), 
                    "path_index_: %d / %zu, Matches: %zu Target Yaw: %f Drone Yaw: %f | FPS: %.2f (Avg: %.2f)",
                    path_index_, path_data_.size(), good_matches.size(), target_yaw, drone_yaw_, fps_, avg_fps_);
            }
        } else if (DEBUG_) {
            RCLCPP_INFO(this->get_logger(),
                "path_index_: %d / %zu, Matches: %zu Target Yaw: %f Drone Yaw: %f | FPS: %.2f (Avg: %.2f)",
                path_index_, path_history_.size(), good_matches.size(), target_yaw, drone_yaw_, fps_, avg_fps_);
        }
    }
}

}  // namespace gps_denied_nav
