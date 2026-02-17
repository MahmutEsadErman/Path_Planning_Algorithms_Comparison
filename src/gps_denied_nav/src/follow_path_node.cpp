/**
 * @file follow_path_node.cpp
 * @brief Constructor, destructor, path loading, and utility functions for FollowPathNode
 * 
 * Other implementations are split across:
 *   - path_following.cpp: imageCallback, alignToTarget, followPath
 *   - yaw_controller.cpp: PID controller, yaw filter, velocity publishing
 *   - flight_report.cpp:  generateFlightReport
 */

#include "gps_denied_nav/follow_path_node.hpp"
#include <cmath>
#include <algorithm>

namespace gps_denied_nav {

// ========== Constructor ==========

FollowPathNode::FollowPathNode() : Node("follow_path_node")
{
    // Initialize state
    path_index_ = 0;
    ready_ = false;
    lost_path_ = false;
    returning_ = false;

    // Declare parameters
    this->declare_parameter<std::string>("path_file", "yeni_harita_SURF.yaml");
    this->declare_parameter<std::string>("report_output_file", "flight_report.csv");
    this->declare_parameter<double>("camera_pitch_angle", 90.0);
    this->declare_parameter<int>("similarity_threshold", 100);
    this->declare_parameter<int>("min_feature_count", 40);
    this->declare_parameter<double>("yaw_kp", 0.4);
    this->declare_parameter<double>("yaw_ki", 0.001);
    this->declare_parameter<double>("yaw_kd", 0.0);
    this->declare_parameter<double>("yaw_integral_limit", 0.8);
    this->declare_parameter<double>("yaw_output_limit", 0.6);
    this->declare_parameter<double>("velocity", 5.0);
    this->declare_parameter<bool>("debug", true);

    // Get parameters
    similarity_threshold_ = this->get_parameter("similarity_threshold").as_int();
    min_feature_count_ = this->get_parameter("min_feature_count").as_int();
    vel_ = this->get_parameter("velocity").as_double();
    camera_pitch_angle_ = this->get_parameter("camera_pitch_angle").as_double();
    path_file_ = this->get_parameter("path_file").as_string();
    report_output_file_ = this->get_parameter("report_output_file").as_string();
    DEBUG_ = this->get_parameter("debug").as_bool();
    yaw_kp_ = this->get_parameter("yaw_kp").as_double();
    yaw_ki_ = this->get_parameter("yaw_ki").as_double();
    yaw_kd_ = this->get_parameter("yaw_kd").as_double();
    yaw_integral_limit_ = std::max(0.0, this->get_parameter("yaw_integral_limit").as_double());
    yaw_output_limit_ = std::max(0.0, this->get_parameter("yaw_output_limit").as_double());

    // Create marker publisher
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/last_point_marker", 10);

    // Load path from file
    loadPath(path_file_);

    // Initialize feature processor
    feature_processor_ = std::make_unique<FeatureProcessor>(feature_detector_, K_, camera_pitch_angle_);

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

// ========== Destructor ==========

FollowPathNode::~FollowPathNode()
{
    generateFlightReport();
}

// ========== Utility Functions ==========

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

// ========== Simple Callback Functions ==========

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

}  // namespace gps_denied_nav
