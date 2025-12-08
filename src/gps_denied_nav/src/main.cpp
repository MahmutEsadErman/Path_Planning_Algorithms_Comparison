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
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/position_target.hpp>
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
    int path_index_;
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_;
    mavros_msgs::msg::PositionTarget msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool timer_started_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    double similarity_threshold;
    cv::Mat K_;
    std::string feature_detector;
    cv::Mat cam_tf;
    double drone_yaw;


    FollowPathNode() : Node("follow_path_node"), timer_started_(false)
    {   
        // Initialize path index
        path_index_ = 0;

        this->declare_parameter<std::string>("path_file", "simple_path.yaml");
        this->declare_parameter<double>("camera_pitch_angle", 60.0);
        this->declare_parameter<int>("similarity_threshold", 50);
        this->declare_parameter<double>("yaw_kp", 0.01);
        this->declare_parameter<double>("pitch_kp", 1);
        
        similarity_threshold = this->get_parameter("similarity_threshold").as_int();
        
        load_path(this->get_parameter("path_file").as_string());

        // Initialize feature detector and FLANN matcher
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

        // subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            rclcpp::SensorDataQoS(),
            std::bind(&FollowPathNode::image_callback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&FollowPathNode::imu_callback, this, std::placeholders::_1)
        );

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/target_pose",
            10
        );

        pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        // Step 1: Define C_Cros_Ccv (OpenCV Cam to ROS-style Cam)
        // OpenCV (Ccv): X-right, Y-down, Z-forward
        // ROS-style (Cros): X-forward, Y-left, Z-up
        cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,   // ROS X = CV Z
            -1,  0,  0,   // ROS Y = -CV X
             0, -1,  0);  // ROS Z = -CV Y

        // Step 2: Define C_B_Cros (ROS-style Cam to Drone Body)
        // This is the static camera pitch angle around the Y-axis.
        double camera_pitch_angle = this->get_parameter("camera_pitch_angle").as_double();
        double angle_rad = camera_pitch_angle * M_PI / 180.0;
        cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
            cos(angle_rad), 0, sin(angle_rad),
                         0, 1,              0,
           -sin(angle_rad), 0, cos(angle_rad)
        );
        
        // Step 3: Combine them to get C_B_Ccv (OpenCV Cam to Drone Body)
        cam_tf = C_B_Cros * C_Cros_Ccv;
        
        follow_path(0, 0);
    }

    void vel_publisher(geometry_msgs::msg::Twist vel, double yaw_rate)
    {   
        msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Ignore position, acceleration, and absolute yaw
        // Enable: velocity (vx, vy, vz) and yaw_rate
        msg_.type_mask =
              mavros_msgs::msg::PositionTarget::IGNORE_PX
            | mavros_msgs::msg::PositionTarget::IGNORE_PY
            | mavros_msgs::msg::PositionTarget::IGNORE_PZ
            | mavros_msgs::msg::PositionTarget::IGNORE_AFX
            | mavros_msgs::msg::PositionTarget::IGNORE_AFY
            | mavros_msgs::msg::PositionTarget::IGNORE_AFZ
            | mavros_msgs::msg::PositionTarget::IGNORE_YAW;  // Ignore absolute yaw, use yaw_rate instead

        msg_.velocity = vel.linear;
        msg_.yaw_rate = yaw_rate;  // rad/s - positive = counter-clockwise (left)

        pub_->publish(msg_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {   
        // Calculate yaw
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        drone_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        double target_yaw = 0;
        static auto start = std::chrono::high_resolution_clock::now();
        std::cout << "Feature comparison time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms" << std::endl;

        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        std::vector<cv::DMatch> good_matches;
         
        fe_method->detectAndCompute(gray_image, cv::noArray(), kp, des);
        int matches = compare_features(path_data_[path_index_].features.descriptors, des, good_matches);
        
        // Only calculate translation if we have enough good matches
        if (good_matches.size() >= 10) {
            start = std::chrono::high_resolution_clock::now();
            target_yaw = calculate_t_with_features(kp, path_data_[path_index_].features.keypoints, good_matches);
            std::cout << "Translation calculation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms" << std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "Not enough matches (%zu) for pose estimation", good_matches.size());
        }

        RCLCPP_INFO(this->get_logger(), "path_index_: %d, Matches: %d Target Yaw: %f Drone Yaw: %f", (int)path_index_, matches, target_yaw, drone_yaw);
        if (matches > similarity_threshold) {
            // Increment path index
            path_index_ += 1;
            pose_publisher_->publish(path_data_[path_index_].target_pose);
            if (path_index_ >= path_data_.size()) {
                RCLCPP_INFO(this->get_logger(), "Path following completed");
                return;
            }
        }
        follow_path(matches, target_yaw);
        start = std::chrono::high_resolution_clock::now();
    }

    void follow_path(int matches, double target_yaw)
    {   
        static int prev_index = -1;  // Use signed int to allow -1 as initial value
        static double yaw_kp = this->get_parameter("yaw_kp").as_double();
        static double vel = this->get_parameter("pitch_kp").as_double();
        static double prev_similarity = 0;
        static double vel_x = 0;
        static double vel_y = 0;
        static double vel_z = 0;
        static double altitude = 0;
        
        
        if (path_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path data is empty");
            return;
        }

        if (path_index_ > prev_index) {
            RCLCPP_INFO(this->get_logger(), "path_index_: %d", (int)path_index_);
            const FrameData &current_frame = path_data_[path_index_];
            prev_index = path_index_;
            // // Convert quaternion to roll/pitch/yaw
            // double qx = current_frame.imu.orientation.x;
            // double qy = current_frame.imu.orientation.y;
            // double qz = current_frame.imu.orientation.z;
            // double qw = current_frame.imu.orientation.w;
            // // Calculate yaw
            // double siny_cosp = 2.0 * (qw * qz + qx * qy);
            // double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            // // target_yaw = std::atan2(siny_cosp, cosy_cosp);

            altitude = current_frame.altitude.data;

            vel_x = vel;
        }
        // else {
        //     vel_x -= pitch_kp;
        // }

        // if (target_yaw > M_PI/2 || target_yaw < 3*M_PI/2) {
        //     pitch_kp = -pitch_kp;
        // }
        
        vel_x = std::max(-5.0, std::min(5.0, vel_x));
        // prev_similarity = similarity;

        target_yaw = yaw_kp * target_yaw;

        // Manually rotate body-frame velocity (vel_x=forward, vel_y=left) 
        
        double cos_yaw = std::cos(drone_yaw);
        double sin_yaw = std::sin(drone_yaw);
        
        // Apply rotation: body -> global ENU
        double global_vx = cos_yaw * vel_x - sin_yaw * vel_y;
        double global_vy = sin_yaw * vel_x + cos_yaw * vel_y;
        double global_vz = vel_z;  // Z velocity is unaffected by yaw rotation

        // Create and publish manual control message with rotated velocities
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = global_vx;
        vel_msg.linear.y = global_vy;
        vel_msg.linear.z = global_vz;
        vel_publisher(vel_msg, target_yaw);
        RCLCPP_INFO(this->get_logger(), 
                    "Body: vx=%.2f, vy=%.2f | yaw=%.2f rad, yaw_rate=%.2f rad/s", 
                    vel_x, vel_y, drone_yaw, target_yaw);
    }

    void load_path(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for reading: %s", filename.c_str());
            return;
        }

        fs["K"] >> K_;
        fs["feature_detector"] >> feature_detector;

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

    int compare_features(const cv::Mat& des1 , const cv::Mat& des2, std::vector<cv::DMatch> &good_matches)
    {
        if (des1.rows < 2 || des2.rows < 2)
        {
            // std::cerr << "Warning: No descriptors found or not enough for knnMatch." << std::endl;
            return 0;
        }
        
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(des1, des2, matches, 2); // k=2 for ratio test
        
        // 3. Ratio Test
        float ratio_test_k = 0.75f;
        for (const auto& match_pair : matches)
        {
            if (match_pair.size() == 2 && match_pair[0].distance < ratio_test_k * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        int match_size = static_cast<int>(good_matches.size());

        return match_size;
    }

    double calculate_t_with_features(const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2, const std::vector<cv::DMatch>& good_matches)
    {
        // 4. Get corresponding points
        std::vector<cv::Point2f> q1, q2;
        for(const auto& m : good_matches)
        {
            q1.push_back(kp1[m.queryIdx].pt);
            q2.push_back(kp2[m.trainIdx].pt);
        }

        // 5. Estimate motion   
        cv::Mat E, R, t, mask;
        
        // Use USAC_MAGSAC (modern RANSAC) and stricter threshold (0.5 px)
        E = cv::findEssentialMat(q1, q2, K_, cv::USAC_MAGSAC, 0.999, 0.2, mask);
        
        // Pass the mask to recoverPose so it uses only the good inliers
        cv::recoverPose(E, q1, q2, K_, R, t, mask);
        
        // Transform translation: t_body = C * t_cam
        cv::Mat t_ros = cam_tf * t;

        std::cout << "Translation: " << t_ros << std::endl;

        // Calculate target angle from translation vector
        // atan2(y, x) where y=Y_component (left), x=X_component (forward)
        double target_angle = std::atan2(t_ros.at<double>(1), t_ros.at<double>(0));

        // cv::drawMatches(frame1, kp1, frame2, kp2, good_matches, img_matches);
        // match_size = good_matches.size();

        return target_angle;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}