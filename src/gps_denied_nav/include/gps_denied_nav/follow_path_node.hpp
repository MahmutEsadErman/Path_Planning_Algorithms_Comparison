/**
 * @file follow_path_node.hpp
 * @brief ROS2 Node for controlling drone using visual path following
 */

#ifndef GPS_DENIED_NAV_FOLLOW_PATH_NODE_HPP
#define GPS_DENIED_NAV_FOLLOW_PATH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "gps_denied_nav/types.hpp"
#include "gps_denied_nav/feature_processor.hpp"

#include <chrono>
#include <deque>
#include <memory>
#include <fstream>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace gps_denied_nav {

/**
 * @brief Per-frame timing measurements in milliseconds
 */
struct FrameTiming {
    double feature_extraction_ms = 0.0;
    double feature_matching_ms = 0.0;
    double yaw_estimation_ms = 0.0;
    double yaw_filtering_ms = 0.0;
    double total_frame_ms = 0.0;
};

/**
 * @brief Per-frame metrics snapshot for flight report
 */
struct FrameMetrics {
    // Position (ground truth)
    double gt_x = 0.0;
    double gt_y = 0.0;

    // Orientation
    double drone_yaw = 0.0;

    // Path state
    int path_index = 0;
    bool is_returning = false;

    // Feature matching
    int good_match_count = 0;

    // Timing
    FrameTiming timing;
};

/**
 * @brief ROS2 Node for GPS-denied visual path following
 */
class FollowPathNode : public rclcpp::Node {
public:
    FollowPathNode();
    ~FollowPathNode();

private:
    // ========== Callback Functions ==========
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gtPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // ========== Path Following Functions ==========
    void alignToTarget(const cv::Mat& target_descriptors,
                       const std::vector<cv::KeyPoint>& target_keypoints,
                       const cv::Mat& current_descriptors,
                       const std::vector<cv::KeyPoint>& current_keypoints,
                       bool is_initial_alignment = false);
    void followPath(double vel, double target_yaw);
    int getTraversalPointCount() const;
    const geometry_msgs::msg::Pose* getTraversalPose(int traversal_index) const;
    double computeUpcomingTurnAngle() const;
    double computeTurnSpeedScale(double turn_angle_rad) const;

    // ========== Velocity Publishing Functions ==========
    void velPublish(geometry_msgs::msg::Twist vel, double yaw);
    void yawPublish(double yaw);

    // ========== Utility Functions ==========
    void loadPath(const std::string& filename);
    void publishLastPointMarker();

    // ========== Flight Report Functions ==========
    void generateFlightReport();

    // ========== Yaw PID Functions ==========
    double runYawPid(double yaw_error);
    void resetYawPid();
    
    // ========== Yaw Filter Functions ==========
    double filterYaw(double raw_yaw);
    void clearYawFilter();

    // ========== Member Variables ==========
    
    // Path data
    std::vector<FrameData> path_data_;
    std::vector<std::pair<Features, geometry_msgs::msg::Pose>> path_history_;
    int path_index_;
    
    // Pose and orientation
    std::shared_ptr<geometry_msgs::msg::Pose> gt_pose_;
    double starting_yaw_ = 0.0;
    geometry_msgs::msg::Point starting_position_;
    double drone_yaw_ = 0.0;
    double current_vel_magnitude_ = 0.0;

    // Timing
    std::chrono::time_point<std::chrono::high_resolution_clock> path_following_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_time_;

    // FPS calculation
    double fps_ = 0.0;
    double avg_fps_ = 0.0;
    int frame_count_ = 0;
    double fps_sum_ = 0.0;
    bool first_frame_ = true;

    // Match buffer
    int match_size_buff_[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int match_buff_sum_ = 0;
    int match_frame_index_ = 0;

    // Endpoint
    double distance_to_endpoint_ = -1.0;
    bool path_completed_ = false;

    // Per-frame metrics data (for flight report)
    std::vector<FrameMetrics> metrics_data_;
    std::string report_output_file_ = "flight_report.csv";
    int path_loss_count_ = 0;

    // Yaw filter (EMA-based)
    bool filter_initialized_ = false;
    double filtered_yaw_ = 0.0;
    int outlier_count_ = 0;

    // Yaw PID state
    bool yaw_pid_initialized_ = false;
    double yaw_integral_ = 0.0;
    double prev_yaw_error_ = 0.0;
    std::chrono::time_point<std::chrono::steady_clock> last_yaw_pid_time_;

    // Flags
    bool DEBUG_;
    bool ready_ = false;
    bool lost_path_ = false;
    bool returning_ = false;

    // Parameters
    std::string path_file_;
    int similarity_threshold_;
    int min_feature_count_;
    double camera_pitch_angle_;
    double vel_;
    std::string feature_detector_;
    double yaw_kp_ = 0.05;
    double yaw_ki_ = 0.0;
    double yaw_kd_ = 0.0;
    double yaw_integral_limit_ = 0.8;
    double yaw_output_limit_ = 0.6;
    bool turn_slowdown_enabled_ = true;
    double turn_slowdown_start_rad_ = 0.349066;  // 20 deg
    double turn_slowdown_full_rad_ = 1.22173;    // 70 deg
    double turn_slowdown_min_ratio_ = 0.35;

    // Feature processor
    std::unique_ptr<FeatureProcessor> feature_processor_;

    // Camera matrices
    cv::Mat K_;
    cv::Mat cam_tf_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr returning_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gt_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    // Message buffer
    mavros_msgs::msg::PositionTarget msg_;
};

}  // namespace gps_denied_nav

#endif  // GPS_DENIED_NAV_FOLLOW_PATH_NODE_HPP
