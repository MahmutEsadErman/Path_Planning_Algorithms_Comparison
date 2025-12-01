/**
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <cmath>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cv_bridge/cv_bridge.hpp>

struct Features {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct FrameData {
    Features features;
    sensor_msgs::msg::Imu imu;
    std_msgs::msg::Float64 altitude;
    geometry_msgs::msg::Pose target_pose;
};

class FollowPathNode : public rclcpp::Node {
public:
    // Member variables
    std::vector<FrameData> path_data_;
    size_t path_index_;
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    bool timer_started_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    double similarity_threshold;

    FollowPathNode() : Node("follow_path_node"), timer_started_(false)
    {   
        // Initialize path index
        path_index_ = 0;

        this->declare_parameter<double>("similarity_threshold", 0.5);
        similarity_threshold = this->get_parameter("similarity_threshold").as_double();

        // Initialize feature detector and FLANN matcher
        std::string feature_detector = "SURF";

        if (feature_detector == "ORB") {
            fe_method = cv::ORB::create();
            // Use LSH Index for binary descriptors (ORB)
            // Parameters: table_number=12, key_size=20, multi_probe_level=2
            matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        } else if (feature_detector == "SURF") {
            int hessian_threshold = 400;
            fe_method = cv::xfeatures2d::SURF::create(hessian_threshold);
            matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>(5));
        } else {
            // Default to SIFT
            fe_method = cv::SIFT::create();
            // Use KD-Tree Index for floating point descriptors (SIFT)
            matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>(5));
        }

        // publishers
        manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
            "/drone/cmd_move",
            10
        );
        pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
            "/target_pose",
            10
        );

        // subscribers
        this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            10,
            std::bind(&FollowPathNode::image_callback, this, std::placeholders::_1)
        );
              
        load_path("saved_path.yaml");
        
        // Wait for user input before starting
        RCLCPP_INFO(this->get_logger(), "Path created. Type 'y' and press Enter to start following the path:");
        std::string input;
        std::getline(std::cin, input);
        if (input == "y" || input == "Y") {
            // RCLCPP_INFO(this->get_logger(), "Starting path following...");
            // timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(50), // 30 Hz
            //     std::bind(&FollowPathNode::follow_path, this)
            // );
            // timer_started_ = true;
            follow_path();
        } else {
            RCLCPP_WARN(this->get_logger(), "Path following not started. Expected 'y' but got: %s", input.c_str());
        }

    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        fe_method->detectAndCompute(gray_image, cv::noArray(), kp, des);
        double similarity = compare_features(path_data_[path_index_].features.descriptors, des);
        std::cout << "Similarity: " << similarity << std::endl;
        if (similarity > similarity_threshold) {
            // Increment path index
            path_index_ += 1;
        }
        follow_path();
    }

    void follow_path()
    {

        if (path_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path data is empty");
            return;
        }

        // Loop through the path data
        const FrameData &current_frame = path_data_[path_index_];

        // Convert quaternion to roll/pitch/yaw
        double qx = current_frame.imu.orientation.x;
        double qy = current_frame.imu.orientation.y;
        double qz = current_frame.imu.orientation.z;
        double qw = current_frame.imu.orientation.w;

        double roll = 0;
        double pitch = 50;

        // Calculate yaw
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // Create and publish manual control message
        mavros_msgs::msg::ManualControl manual_msg;
        manual_msg.x = pitch;  // Pitch value
        manual_msg.y = roll;   // Roll value
        manual_msg.z = current_frame.altitude.data;  // Altitude value
        manual_msg.r = yaw;    // Yaw value
        manual_pub_->publish(manual_msg);
        RCLCPP_INFO(this->get_logger(), "p: %.2f, r: %.2f, y: %.2f, alt: %.2f", pitch, roll, yaw, current_frame.altitude.data);
    }

    void load_path(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for reading: %s", filename.c_str());
            return;
        }

        cv::FileNode frames = fs["frames"];
        if (frames.type() != cv::FileNode::SEQ) {
            RCLCPP_ERROR(this->get_logger(), "Invalid file format: 'frames' is not a sequence");
            return;
        }

        for (auto it = frames.begin(); it != frames.end(); ++it) {
            cv::FileNode frame_node = *it;
            FrameData frame;

            // Keypoints
            frame_node["keypoints"] >> frame.features.keypoints;
            // Descriptors
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
                cv::FileNode pose_node = frame_node["target_pose"];
                cv::FileNode pos_node = pose_node["position"];
                frame.target_pose.position.x = (double)pos_node["x"];
                frame.target_pose.position.y = (double)pos_node["y"];
                frame.target_pose.position.z = (double)pos_node["z"];

                cv::FileNode rot_node = pose_node["orientation"];
                frame.target_pose.orientation.x = (double)rot_node["x"];
                frame.target_pose.orientation.y = (double)rot_node["y"];
                frame.target_pose.orientation.z = (double)rot_node["z"];
                frame.target_pose.orientation.w = (double)rot_node["w"];
            }

            path_data_.push_back(frame);
        }
        
        fs.release();
        RCLCPP_INFO(this->get_logger(), "Loaded path with %zu frames", path_data_.size());
    }

    double compare_features(const cv::Mat& des1 , const cv::Mat& des2)
    {
        if (des1.empty() || des2.rows < 2)
        {
            // std::cerr << "Warning: No descriptors found or not enough for knnMatch." << std::endl;
            return 0.0;
        }
        
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(des1, des2, matches, 2); // k=2 for ratio test

        // 3. Ratio Test
        std::vector<cv::DMatch> good_matches;
        float ratio_test_k = 0.6f;
        for (const auto& match_pair : matches)
        {
            if (match_pair.size() == 2 && match_pair[0].distance < ratio_test_k * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        double match_size = static_cast<double>(good_matches.size());

        return match_size / (double)std::min(des1.rows, des2.rows);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}