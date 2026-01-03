/*
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
    std::vector<std::pair<Features, geometry_msgs::msg::Pose>> path_history_;
    int path_index_;
    std::shared_ptr<geometry_msgs::msg::Pose> gt_pose_;
    double starting_yaw_;
    geometry_msgs::msg::Point starting_position_;
    std::chrono::time_point<std::chrono::high_resolution_clock> path_following_time_;
    double drone_yaw = 0.0;
    double current_vel_magnitude_ = 0.0;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr returning_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gt_pose__sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    mavros_msgs::msg::PositionTarget msg_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    int similarity_threshold;
    int min_feature_size;
    cv::Mat K_;
    cv::Mat cam_tf;
    std::string feature_detector;
    double vel_;

    // Flags
    bool DEBUG;
    bool ready_;
    bool lost_path_;
    bool returning_;

    FollowPathNode() : Node("follow_path_node")
    {   
        // Initialize path index
        path_index_ = 0;

        // Initialize ready flag
        ready_ = false;
        lost_path_ = false;
        returning_ = false;

        this->declare_parameter<std::string>("path_file", "path_90degree_surf.yaml");
        this->declare_parameter<double>("camera_pitch_angle", 90.0);
        this->declare_parameter<int>("similarity_threshold", 63);
        this->declare_parameter<int>("min_feature_size", 20);
        this->declare_parameter<double>("yaw_kp", 0.05);
        this->declare_parameter<double>("velocity", 5);
        this->declare_parameter<bool>("debug", true);
        
        similarity_threshold = this->get_parameter("similarity_threshold").as_int();
        min_feature_size = this->get_parameter("min_feature_size").as_int();
        vel_ = this->get_parameter("velocity").as_double();
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

        gt_pose__sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info",
            rclcpp::SensorDataQoS(),
            std::bind(&FollowPathNode::gt_pose__callback, this, std::placeholders::_1)
        );

        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local",
            rclcpp::SensorDataQoS(),
            std::bind(&FollowPathNode::vel_callback, this, std::placeholders::_1)
        );

        // publishers
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/target_pose",
            10
        );

        returning_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/returning_status",
            10
        );

        vel_publisher_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
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

        vel_publisher_->publish(msg_);
    }

    void yaw_publish(double yaw)
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

        msg_.yaw = yaw;

        vel_publisher_->publish(msg_);
    }

    void turn_to_starting_yaw()
    {   
        if (!lost_path_) {
            double qx = path_data_[path_data_.size()-1].imu.orientation.x;
            double qy = path_data_[path_data_.size()-1].imu.orientation.y;
            double qz = path_data_[path_data_.size()-1].imu.orientation.z;
            double qw = path_data_[path_data_.size()-1].imu.orientation.w;
            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            starting_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        }
        yaw_publish(starting_yaw_);

        if (DEBUG) {
            std::cout << "Starting yaw: " << starting_yaw_ << std::endl;
        }
        
        if (std::abs(drone_yaw - starting_yaw_) < 0.05) {
            // save starting position for error calculations
            starting_position_ = gt_pose_->position;

            // Publish the last point as a marker in RViz
            publish_last_point_marker();
            
            // initialize this to calculate path following time 
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

        double x = gt_pose_->position.x - starting_position_.x;
        double y = gt_pose_->position.y - starting_position_.y;

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

    void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {   
        double vx = msg->twist.linear.x;
        double vy = msg->twist.linear.y;
        double vz = msg->twist.linear.z;
        current_vel_magnitude_ = std::sqrt(vx*vx + vy*vy + vz*vz);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        double target_yaw = 0;
        static int match_size_buff[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
        static int match_buff_sum = 0;
        static int frame_index = 0;
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        std::vector<cv::DMatch> good_matches;
        static double vel = vel_;
        
        if (!returning_) {
            // If the uav is not turning to the starting yaw
            if (!ready_) {
                turn_to_starting_yaw();
                return;
            }

            // Detect features
            fe_method->detectAndCompute(gray_image, cv::noArray(), kp, des);

            // Compare features with the last frame (only if path_history_ is not empty)
            std::vector<cv::DMatch> last_frame_matches;
            if (!path_history_.empty()) {
                compare_features(path_history_.back().first.descriptors, des, last_frame_matches);
            }

            // Save path history - initialize with first frame or add if different enough
            if (kp.size() >= static_cast<size_t>(similarity_threshold + 10) && 
                (path_history_.empty() || last_frame_matches.size() < static_cast<size_t>(similarity_threshold))) {
                    Features frame;
                    frame.keypoints = kp;
                    frame.descriptors = des;
                    path_history_.push_back(std::pair<Features, geometry_msgs::msg::Pose>(frame, *gt_pose_));
            }

            if (lost_path_) {
                yaw_publish(drone_yaw);
                if (current_vel_magnitude_ < 0.5) {
                    path_index_ = 0;
                    returning_ = true;
                    ready_ = false;
                    starting_yaw_ = drone_yaw + M_PI;
                    
                    // publish returning message
                    std_msgs::msg::Bool bool_msg;
                    bool_msg.data = true;
                    returning_publisher_->publish(bool_msg);

                    RCLCPP_INFO(this->get_logger(), "Stopped. Now turning around.");
                    RCLCPP_INFO(this->get_logger(), "path history size: %ld", path_history_.size());
                }
                return;
            }

            // Compare features with the current path point
            compare_features(path_data_[path_index_].features.descriptors, des, good_matches);
            
            // Only calculate translation if we have enough good matches
            if (good_matches.size() >= min_feature_size) {            
                target_yaw = calculate_t_with_features(path_data_[path_index_].features.keypoints, kp, good_matches);
            } 
            // If we lost the path, first stop then go back to home
            else if (match_size_buff[9] != -1 && match_buff_sum/10 < min_feature_size){ 
                lost_path_ = true;
                RCLCPP_INFO(this->get_logger(), "\n---Lost the path. Stopping...---\n");
            }

            if (good_matches.size() > similarity_threshold) {
                if (path_index_ >= path_data_.size()-1) {
                    vel = 0;
                    image_sub_.reset();
                    double avg_error = calculate_error() / path_data_.size();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - path_following_time_).count();
                    RCLCPP_INFO(this->get_logger(), "Path following completed. \n Average error: %f m", avg_error);
                    RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                    return;
                }

                calculate_error();

                // Increment path index
                path_index_ += 1;
                target_pose_publisher_->publish(path_data_[path_index_].target_pose);
            }
        }
        else { // Go back to home
            // Turn yaw to opposite direction
            if (!ready_) {
                turn_to_starting_yaw();
                return;
            }

            if (path_history_.empty()) {
                RCLCPP_WARN(this->get_logger(), "Path history is empty, cannot backtrack.");
                return;
            }

            int history_idx = path_history_.size() - 1 - path_index_;

            // Detect and compare features
            fe_method->detectAndCompute(gray_image, cv::noArray(), kp, des);
            compare_features(path_history_[history_idx].first.descriptors, des, good_matches);
            
            // Only calculate translation if we have enough good matches
            if (good_matches.size() >= min_feature_size) {            
                target_yaw = calculate_t_with_features(path_history_[history_idx].first.keypoints, kp, good_matches);
            } 

            if (good_matches.size() > similarity_threshold) {
                if (path_index_ >= path_history_.size()-1) {
                    vel = 0;
                    image_sub_.reset();
                    double avg_error = calculate_error() / path_history_.size();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - path_following_time_).count();
                    RCLCPP_INFO(this->get_logger(), "Backtracking completed. \n Average error: %f m", avg_error);
                    RCLCPP_INFO(this->get_logger(), "Time taken: %ld s", elapsed_time);
                    return;
                }

                calculate_error();

                // Increment path index
                path_index_ += 1;
            }

        }

        if (path_index_ > 1) {
            follow_path(vel, target_yaw);
            if (!lost_path_) {
                match_buff_sum -= match_size_buff[frame_index];
                match_buff_sum += good_matches.size();
                match_size_buff[frame_index] = good_matches.size();
                frame_index = (frame_index+1)%10;
            }
            if (DEBUG) {
                RCLCPP_INFO(this->get_logger(), "path_index_: %d, Matches: %ld Target Yaw: %f Drone Yaw: %f", (int)path_index_, good_matches.size(), target_yaw, drone_yaw);
            }
        }
    }

    void follow_path(double vel, double target_yaw)
    {   
        static double yaw_kp = this->get_parameter("yaw_kp").as_double();
        double vel_x = vel;
        double vel_y = 0;
        double vel_z = 0;

        if (DEBUG) {
            RCLCPP_INFO(this->get_logger(), 
                        "yaw=%.2f rad, target_yaw=%.2f rad", 
                        drone_yaw, target_yaw);
        }

        target_yaw = yaw_kp * target_yaw + drone_yaw;

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

        // Calculate target angle from translation vector
        // atan2(y, x) where y=Y_component (left), x=X_component (forward)
        double target_angle = std::atan2(t_ros.at<double>(1), t_ros.at<double>(0));

        // cv::drawMatches(frame1, kp1, frame2, kp2, good_matches, img_matches);
        // match_size = good_matches.size();

        return target_angle;
    }

    void gt_pose__callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Assuming the desired pose is the third one in the array
        if (msg->poses.size() > 2) {
            gt_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[2]);
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