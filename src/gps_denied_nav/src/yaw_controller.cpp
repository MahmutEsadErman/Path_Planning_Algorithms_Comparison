/**
 * @file yaw_controller.cpp
 * @brief Yaw PID controller, yaw filter, and velocity publishing functions
 */

#include "gps_denied_nav/follow_path_node.hpp"
#include <cmath>
#include <algorithm>

namespace gps_denied_nav {

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

// ========== Yaw PID ==========

double FollowPathNode::runYawPid(double yaw_error)
{
    const double integral_limit = yaw_integral_limit_;
    const double output_limit = yaw_output_limit_;

    if (output_limit <= 0.0) {
        return 0.0;
    }

    const auto now = std::chrono::steady_clock::now();
    double dt = 0.0;
    if (yaw_pid_initialized_) {
        dt = std::chrono::duration<double>(now - last_yaw_pid_time_).count();
    } else {
        yaw_pid_initialized_ = true;
    }
    last_yaw_pid_time_ = now;

    if (dt <= 0.0 || dt > 0.5) {
        dt = 0.0;
    }

    if (dt > 0.0 && yaw_ki_ != 0.0) {
        yaw_integral_ += yaw_error * dt;
        yaw_integral_ = std::clamp(yaw_integral_, -integral_limit, integral_limit);
    }

    double derivative = 0.0;
    if (dt > 0.0) {
        derivative = (yaw_error - prev_yaw_error_) / dt;
    }
    prev_yaw_error_ = yaw_error;

    const double output = yaw_kp_ * yaw_error + yaw_ki_ * yaw_integral_ + yaw_kd_ * derivative;
    return std::clamp(output, -output_limit, output_limit);
}

void FollowPathNode::resetYawPid()
{
    yaw_pid_initialized_ = false;
    yaw_integral_ = 0.0;
    prev_yaw_error_ = 0.0;
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

    double diff = std::abs(raw_yaw - filtered_yaw_);

    if (diff > 1.5) {  // ~86 degrees tolerance
        outlier_count_++;
        if (DEBUG_) {
            RCLCPP_WARN(this->get_logger(), "Outlier rejected: %.2f rad (diff: %.2f, count: %d)",
                        raw_yaw, diff, outlier_count_);
        }

        if (outlier_count_ > 10) {
            RCLCPP_WARN(this->get_logger(), "Too many outliers, resetting filter");
            filter_initialized_ = false;
        }

        return filtered_yaw_;
    }

    outlier_count_ = 0;

    // Exponential Moving Average
    double alpha = 0.4;
    filtered_yaw_ = alpha * raw_yaw + (1.0 - alpha) * filtered_yaw_;

    return filtered_yaw_;
}

void FollowPathNode::clearYawFilter()
{
    filter_initialized_ = false;
    filtered_yaw_ = 0.0;
    outlier_count_ = 0;
}

}  // namespace gps_denied_nav
