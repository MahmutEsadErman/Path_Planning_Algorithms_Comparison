/**
 * @file follow_path_node.hpp
 * @brief ROS2 Node for controlling drone using visual path following
 */

#ifndef GPS_DENIED_NAV_FOLLOW_PATH_NODE_HPP
#define GPS_DENIED_NAV_FOLLOW_PATH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
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

namespace gps_denied_nav {

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
    int searchForwardPath(const cv::Mat& current_descriptors, int start_index);

    // ========== Velocity Publishing Functions ==========
    void velPublish(geometry_msgs::msg::Twist vel, double yaw);
    void yawPublish(double yaw);

    // ========== Utility Functions ==========
    void loadPath(const std::string& filename);
    void publishLastPointMarker();
    double calculateError();
    
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

    // Error calculation
    double error_sum_ = 0.0;
    int error_count_ = 0;
    double error_calc_m_ = 0.0;
    double error_calc_payda_ = 1.0;
    bool error_calc_initialized_ = false;

    // Match buffer
    int match_size_buff_[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int match_buff_sum_ = 0;
    int match_frame_index_ = 0;

    // Endpoint
    double distance_to_endpoint_ = -1.0;
    bool path_completed_ = false;

    // Yaw filter (EMA-based)
    bool filter_initialized_ = false;
    double filtered_yaw_ = 0.0;
    int outlier_count_ = 0;

    // Forward search
    int forward_search_attempts_ = 0;
    int max_forward_search_range_ = 15;

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
    double yaw_kp_;
    std::string feature_detector_;

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
