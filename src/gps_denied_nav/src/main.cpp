/*
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
#include <visualization_msgs/msg/marker.hpp>
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
    std::shared_ptr<geometry_msgs::msg::Pose> gt_pose;
    double starting_yaw_;
    geometry_msgs::msg::Point starting_position_;
    path_following_time_;


    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gt_pose_sub_;
    mavros_msgs::msg::PositionTarget msg_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    int similarity_threshold;
    cv::Mat K_;
    std::string feature_detector;
    cv::Mat cam_tf;
    double drone_yaw = 0.0;

    // Flags
    bool DEBUG;
    bool ready_;

    FollowPathNode() : Node("follow_path_node")
    {   
        // Initialize path index
        path_index_ = 0;

        // Initialize ready flag
        ready_ = false;

        this->declare_parameter<std::string>("path_file", "path_90degree_surf.yaml");
        this->declare_parameter<double>("camera_pitch_angle", 90.0);
        this->declare_parameter<int>("similarity_threshold", 63);
        this->declare_parameter<double>("yaw_kp", 0.05);
        this->declare_parameter<double>("pitch_kp", 5);
        this->declare_parameter<bool>("debug", true);
        
        
        similarity_threshold = this->get_parameter("similarity_threshold").as_int();
        DEBUG = this->get_parameter("debug").as_bool();

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/last_point_marker",
            10
        );
        
        // Load path from file
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

        gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info",
            rclcpp::SensorDataQoS(),
            std::bind(&FollowPathNode::gt_pose_callback, this, std::placeholders::_1)
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
    
    }

    void vel_publish(geometry_msgs::msg::Twist vel, double yaw)
    {   
        msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Ignore position, acceleration, and 
        // Enable: velocity (vx, vy, vz) and yaw
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

        pub_->publish(msg_);
    }

    void turn_to_starting_yaw()
    {   
        // in this function we turn to the starting yaw angle and save starting position for other calculations
        msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Ignore position, acceleration, and 
        // Enable: velocity (vx, vy, vz) and yaw
        msg_.type_mask =
              mavros_msgs::msg::PositionTarget::IGNORE_PX
            | mavros_msgs::msg::PositionTarget::IGNORE_PY
            | mavros_msgs::msg::PositionTarget::IGNORE_PZ
            | mavros_msgs::msg::PositionTarget::IGNORE_AFX
            | mavros_msgs::msg::PositionTarget::IGNORE_AFY
            | mavros_msgs::msg::PositionTarget::IGNORE_AFZ
            | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        // Set velocity to zero while turning
        msg_.velocity.x = 0.0;
        msg_.velocity.y = 0.0;
        msg_.velocity.z = 0.0;

        double qx = path_data_[path_data_.size()-1].imu.orientation.x;
        double qy = path_data_[path_data_.size()-1].imu.orientation.y;
        double qz = path_data_[path_data_.size()-1].imu.orientation.z;
        double qw = path_data_[path_data_.size()-1].imu.orientation.w;
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        starting_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        msg_.yaw = starting_yaw_;

        pub_->publish(msg_);

        std::cout << "Starting yaw: " << starting_yaw_ << std::endl;
        
        if (std::abs(drone_yaw - starting_yaw_) < 0.1) {
            // save starting position for error calculations
            starting_position_ = gt_pose->position;

            // Publish the last point as a marker in RViz
            publish_last_point_marker();

            path_following_time_ = std::chrono::high_resolution_clock::now();

            ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Yaw aligned! Starting path following. Target yaw: %.2f, Drone yaw: %.2f", starting_yaw_, drone_yaw);
        }
    }

    double calculate_error()
    {   
        // Perpendicular distance from drone's position to the path line (through origin at starting_yaw_)
        // Line equation: y = tan(starting_yaw_) * x  =>  -tan(θ)*x + y = 0
        // Distance from point (px, py) to line ax + by + c = 0 is |a*px + b*py + c| / sqrt(a² + b²)
        // Here: a = -tan(θ), b = 1, c = 0
        static double error_sum = 0;
        static double m = std::tan(starting_yaw_);
        static double payda = std::sqrt(m * m + 1);

        double x = gt_pose->position.x - starting_position_.x;
        double y = gt_pose->position.y - starting_position_.y;

        double error = std::abs(-m * x + y) / payda;
        error_sum += error;
        return error_sum;
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
        static auto start = std::chrono::high_resolution_clock::now();
        if (DEBUG) {
            std::cout << "Outside of Image callback time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms" << std::endl;
        }

        if (!ready_) {
            turn_to_starting_yaw();
            return;
        }

        double target_yaw = 0;
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        std::vector<cv::DMatch> good_matches;
        
        // Calculate how much time image processing takes
        if (DEBUG) {
            start = std::chrono::high_resolution_clock::now();
        }

        fe_method->detectAndCompute(gray_image, cv::noArray(), kp, des);
        compare_features(path_data_[path_index_].features.descriptors, des, good_matches);
        
        // Only calculate translation if we have enough good matches
        if (good_matches.size() >= 20) {            
            target_yaw = calculate_t_with_features(path_data_[path_index_].features.keypoints, kp, good_matches);
        } else if (DEBUG){
            RCLCPP_WARN(this->get_logger(), "Not enough matches (%zu) for pose estimation", good_matches.size());
        }

        if (DEBUG) {
            RCLCPP_INFO(this->get_logger(), "path_index_: %d, Matches: %ld Target Yaw: %f Drone Yaw: %f", (int)path_index_, good_matches.size(), target_yaw, drone_yaw);
        }

        if (good_matches.size() > similarity_threshold) {
            if (path_index_ >= path_data_.size()-1) {
                image_sub_.reset();
                double avg_error = calculate_error() / path_data_.size();
                path_following_time_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - path_following_time_).count();
                RCLCPP_INFO(this->get_logger(), "Path following completed. \n Average error: %f m", avg_error);
                RCLCPP_INFO(this->get_logger(), "Time taken: %f s", path_following_time_);
                return;
            }

            calculate_error();
            // Increment path index
            path_index_ += 1;
            pose_publisher_->publish(path_data_[path_index_].target_pose);
        }

        follow_path(good_matches.size(), target_yaw);

        if (DEBUG) {
            std::cout << "image callback time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms" << std::endl;
            start = std::chrono::high_resolution_clock::now();
        }
    }

    void follow_path(int matches, double target_yaw)
    {   
        static double yaw_kp = this->get_parameter("yaw_kp").as_double();
        static double vel = this->get_parameter("pitch_kp").as_double();
        static double vel_x = vel;
        static double vel_y = 0;
        static double vel_z = 0;
        static double altitude = 0;

        if (path_index_ < 1) {
            return;
        }

        if (DEBUG) {
            RCLCPP_INFO(this->get_logger(), 
                        "yaw=%.2f rad, target_yaw=%.2f rad", 
                        drone_yaw, target_yaw);
        }

        // Stopping at the end logic
        if (path_index_ >= path_data_.size()) {
            vel_x = 0;
            vel_y = 0;
            vel_z = 0;
        }

        target_yaw = yaw_kp * target_yaw + drone_yaw;
        // target_yaw = target_yaw * yaw_kp;

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
        vel_publish(vel_msg, target_yaw);
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

    void publish_last_point_marker()
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

        // Get the last point from path_data_
        const auto& last_pose = path_data_[path_data_.size() - 1].target_pose;
        marker.pose.position.x = last_pose.position.x - starting_position_.x;
        marker.pose.position.y = last_pose.position.y - starting_position_.y;
        marker.pose.position.z = starting_position_.z;

        // Set the size of the marker
        marker.scale.x = 20;
        marker.scale.y = 20;
        marker.scale.z = 2;

        // Set the color of the marker
        marker.color.a = 0.5;
        marker.color.r = 0.5;
        marker.color.g = 1.0;
        marker.color.b = 0.5;

        // Make it persist
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means forever

        marker_publisher_->publish(marker);
    }

    void compare_features(const cv::Mat& des1 , const cv::Mat& des2, std::vector<cv::DMatch> &good_matches)
    {
        if (des1.rows < 2 || des2.rows < 2)
            return;
        
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

        if (DEBUG) {
            std::cout << "Translation: " << t_ros << std::endl;
        }

        // Calculate target angle from translation vector
        // atan2(y, x) where y=Y_component (left), x=X_component (forward)
        double target_angle = std::atan2(t_ros.at<double>(1), t_ros.at<double>(0));

        // cv::drawMatches(frame1, kp1, frame2, kp2, good_matches, img_matches);
        // match_size = good_matches.size();

        return target_angle;
    }

    void gt_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Assuming the desired pose is the third one in the array
        if (msg->poses.size() > 2) {
            gt_pose = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[2]);
        } else {
            RCLCPP_WARN(this->get_logger(), "PoseArray does not contain enough poses.");
        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}