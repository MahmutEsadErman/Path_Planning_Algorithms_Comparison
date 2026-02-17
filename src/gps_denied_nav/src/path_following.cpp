/**
 * @file path_following.cpp
 * @brief Image callback, alignment, and path following logic
 */

#include "gps_denied_nav/follow_path_node.hpp"
#include <cmath>
#include <algorithm>

namespace {

double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

namespace gps_denied_nav {

// ========== Alignment ==========

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
        double raw_relative_yaw = feature_processor_->calculateRelativeYaw(
            target_keypoints, current_keypoints, matches);

        relative_yaw = filterYaw(raw_relative_yaw);
        target_yaw = drone_yaw_ + relative_yaw;

        if (DEBUG_) {
            RCLCPP_INFO(this->get_logger(), "Aligning - Raw: %.2f rad, Filtered: %.2f rad (%.1f deg), Drone yaw: %.2f",
                        raw_relative_yaw, relative_yaw, relative_yaw * 180.0 / M_PI, drone_yaw_);
        }
    } else {
        if (DEBUG_) {
            RCLCPP_WARN(this->get_logger(), "Not enough matches for alignment: %zu < %d",
                        matches.size(), min_feature_count_);
        }
        yawPublish(drone_yaw_);
        resetYawPid();
        return;
    }

    followPath(0, relative_yaw);

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
        resetYawPid();
    }
}

// ========== Path Following ==========

void FollowPathNode::followPath(double vel, double target_yaw)
{
    double vel_x = vel;
    double vel_y = 0;
    double vel_z = 0;
    const double yaw_error = normalizeAngle(target_yaw);
    const double yaw_correction = runYawPid(yaw_error);
    const double commanded_yaw = normalizeAngle(drone_yaw_ + yaw_correction);

    if (DEBUG_) {
        RCLCPP_INFO(this->get_logger(),
                    "yaw=%.2f rad, yaw_error=%.2f rad, yaw_cmd=%.2f rad",
                    drone_yaw_, yaw_error, commanded_yaw);
    }

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
    velPublish(vel_msg, commanded_yaw);
}

// ========== Image Callback ==========

void FollowPathNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Start total frame timing
    auto frame_start = std::chrono::high_resolution_clock::now();
    FrameTiming frame_timing;

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
        // Detect features (timed)
        auto t_fe_start = std::chrono::high_resolution_clock::now();
        feature_processor_->detectAndCompute(gray_image, kp, des);
        auto t_fe_end = std::chrono::high_resolution_clock::now();
        frame_timing.feature_extraction_ms = std::chrono::duration<double, std::milli>(t_fe_end - t_fe_start).count();

        // If not aligned yet
        if (!ready_) {
            alignToTarget(
                path_data_[path_index_].features.descriptors,
                path_data_[path_index_].features.keypoints,
                des, kp, true);
            return;
        }

        // Compare with last frame (timed as part of feature matching)
        auto t_hist_match_start = std::chrono::high_resolution_clock::now();
        std::vector<cv::DMatch> last_frame_matches;
        if (!path_history_.empty()) {
            feature_processor_->compareFeatures(
                path_history_.back().first.descriptors, des, last_frame_matches);
        }
        auto t_hist_match_end = std::chrono::high_resolution_clock::now();

        // Save path history
        if (kp.size() >= static_cast<size_t>(similarity_threshold + 10) &&
            (path_history_.empty() || last_frame_matches.size() < static_cast<size_t>(similarity_threshold))) {
            Features frame;
            frame.keypoints = kp;
            frame.descriptors = des;
            path_history_.push_back(std::pair<Features, geometry_msgs::msg::Pose>(frame, *gt_pose_));
        }

        // If path is lost, immediately return home
        if (lost_path_) {
            yawPublish(drone_yaw_);
            resetYawPid();
            if (current_vel_magnitude_ < 0.5) {
                path_index_ = 0;
                returning_ = true;
                ready_ = false;
                resetYawPid();

                std_msgs::msg::Bool bool_msg;
                bool_msg.data = true;
                returning_publisher_->publish(bool_msg);

                RCLCPP_INFO(this->get_logger(), "Path lost. Returning home.");
                RCLCPP_INFO(this->get_logger(), "path history size: %ld", path_history_.size());
            }
            return;
        }

        // Compare features with current path point (timed)
        auto t_match_start = std::chrono::high_resolution_clock::now();
        feature_processor_->compareFeatures(
            path_data_[path_index_].features.descriptors, des, good_matches);
        auto t_match_end = std::chrono::high_resolution_clock::now();
        frame_timing.feature_matching_ms = std::chrono::duration<double, std::milli>(t_match_end - t_match_start).count()
            + std::chrono::duration<double, std::milli>(t_hist_match_end - t_hist_match_start).count();

        if (good_matches.size() >= static_cast<size_t>(min_feature_count_)) {
            // Yaw estimation (timed)
            auto t_yaw_start = std::chrono::high_resolution_clock::now();
            double raw_yaw = feature_processor_->calculateRelativeYaw(
                path_data_[path_index_].features.keypoints, kp, good_matches);
            target_yaw = filterYaw(raw_yaw);
            auto t_yaw_end = std::chrono::high_resolution_clock::now();
            frame_timing.yaw_estimation_ms = std::chrono::duration<double, std::milli>(t_yaw_end - t_yaw_start).count();
        } else if (match_size_buff_[9] != -1 && match_buff_sum_/10 < min_feature_count_) {
            lost_path_ = true;
            path_loss_count_++;
            resetYawPid();
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

                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - path_following_time_).count();
                RCLCPP_INFO(this->get_logger(), "Path following completed.");
                RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                RCLCPP_INFO(this->get_logger(), "Distance to endpoint: %.4f m", distance_to_endpoint_);
                image_sub_.reset();
                return;
            }

            path_index_ += 1;
            target_pose_publisher_->publish(path_data_[path_index_].target_pose);
        }
    } else {
        // Go back to home (timed)
        auto t_fe_start = std::chrono::high_resolution_clock::now();
        feature_processor_->detectAndCompute(gray_image, kp, des);
        auto t_fe_end = std::chrono::high_resolution_clock::now();
        frame_timing.feature_extraction_ms = std::chrono::duration<double, std::milli>(t_fe_end - t_fe_start).count();

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

        auto t_match_start = std::chrono::high_resolution_clock::now();
        feature_processor_->compareFeatures(
            path_history_[history_idx].first.descriptors, des, good_matches);
        auto t_match_end = std::chrono::high_resolution_clock::now();
        frame_timing.feature_matching_ms = std::chrono::duration<double, std::milli>(t_match_end - t_match_start).count();

        if (good_matches.size() >= static_cast<size_t>(min_feature_count_)) {
            auto t_yaw_start = std::chrono::high_resolution_clock::now();
            double raw_yaw = feature_processor_->calculateRelativeYaw(
                path_history_[history_idx].first.keypoints, kp, good_matches);
            target_yaw = filterYaw(raw_yaw);
            auto t_yaw_end = std::chrono::high_resolution_clock::now();
            frame_timing.yaw_estimation_ms = std::chrono::duration<double, std::milli>(t_yaw_end - t_yaw_start).count();
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
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - path_following_time_).count();
                RCLCPP_INFO(this->get_logger(), "Backtracking completed.");
                RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                RCLCPP_INFO(this->get_logger(), "Distance to start point: %.4f m", distance_to_endpoint_);
                return;
            }

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

    // Record per-frame metrics
    auto frame_end = std::chrono::high_resolution_clock::now();
    frame_timing.total_frame_ms = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();

    FrameMetrics metrics;
    if (gt_pose_) {
        metrics.gt_x = gt_pose_->position.x;
        metrics.gt_y = gt_pose_->position.y;
    }
    metrics.drone_yaw = drone_yaw_;
    metrics.path_index = path_index_;
    metrics.is_returning = returning_;
    metrics.good_match_count = static_cast<int>(good_matches.size());
    metrics.timing = frame_timing;
    metrics_data_.push_back(metrics);
}

}  // namespace gps_denied_nav
