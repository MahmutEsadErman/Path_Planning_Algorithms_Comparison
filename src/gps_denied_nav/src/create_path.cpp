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
#include <cmath>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "rclcpp/serialization.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

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

class CreatePathNode : public rclcpp::Node {
public:
    // Member variables
    std::vector<FrameData> path_data_;
    std::shared_ptr<rosbag2_cpp::Reader> reader_;
    bool timer_started_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    double similarity_threshold;
    
    // Feature matching constants
    static constexpr int MIN_DESCRIPTORS_FOR_MATCHING = 2;
    static constexpr float RATIO_TEST_THRESHOLD = 0.7f;
    static constexpr int MIN_GOOD_MATCHES = 10;

    CreatePathNode() : Node("create_path_node"), timer_started_(false)
    {
        this->declare_parameter<std::string>("bag_file_path", "simple_path");
        this->declare_parameter<std::string>("output_file", "simple_path");
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

        // Initialize the ROS2 bag reader
        reader_ = std::make_shared<rosbag2_cpp::Reader>();
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = this->get_parameter("bag_file_path").as_string();
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);

        rclcpp::Time start_time = rclcpp::Clock().now();
        create_path_from_bag();
        rclcpp::Duration elapsed = rclcpp::Clock().now() - start_time;
        std::cout << "Time taken to create path: " << elapsed.seconds() << " seconds" << std::endl;

        save_path(this->get_parameter("output_file").as_string());

    }

    void save_path(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
            return;
        }

        fs << "frames" << "[";
        for (const auto& frame : path_data_) {
            fs << "{";
            
            // Keypoints
            fs << "keypoints" << frame.features.keypoints;
            // Descriptors
            fs << "descriptors" << frame.features.descriptors;
            
            // IMU
            fs << "imu" << "{";
            fs << "linear_acceleration" << "{" 
               << "x" << frame.imu.linear_acceleration.x
               << "y" << frame.imu.linear_acceleration.y
               << "z" << frame.imu.linear_acceleration.z << "}";
            fs << "angular_velocity" << "{"
               << "x" << frame.imu.angular_velocity.x
               << "y" << frame.imu.angular_velocity.y
               << "z" << frame.imu.angular_velocity.z << "}";
            fs << "orientation" << "{"
               << "x" << frame.imu.orientation.x
               << "y" << frame.imu.orientation.y
               << "z" << frame.imu.orientation.z
               << "w" << frame.imu.orientation.w << "}";
            fs << "}";

            // Altitude
            fs << "altitude" << frame.altitude.data;

            // Target Pose
            fs << "target_pose" << "{";
            fs << "position" << "{"
               << "x" << frame.target_pose.position.x
               << "y" << frame.target_pose.position.y
               << "z" << frame.target_pose.position.z << "}";
            fs << "orientation" << "{"
               << "x" << frame.target_pose.orientation.x
               << "y" << frame.target_pose.orientation.y
               << "z" << frame.target_pose.orientation.z
               << "w" << frame.target_pose.orientation.w << "}";
            fs << "}";

            fs << "}";
        }
        fs << "]";
        fs.release();
        RCLCPP_INFO(this->get_logger(), "Path saved to %s", filename.c_str());
    }

    void set_K_from_CameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info_msg)
    {
        K_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) {
            K_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
        }
        K_received_ = true;
    }

    // Helper function to deserialize messages
    template<typename T>
    std::shared_ptr<T> deserializeMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_message = std::make_shared<T>();
        rclcpp::Serialization<T> serialization;
        serialization.deserialize_message(&serialized_msg, ros_message.get());
        return ros_message;
    }

    void create_path_from_bag()
    {
        FrameData frame;
        
        // Accumulators for averaging
        double acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
        double gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
        double alt_sum = 0;
        int imu_count = 0;
        int alt_count = 0;
        geometry_msgs::msg::Quaternion last_orientation;
        
        // Persistent state for handling gaps in data
        double last_valid_altitude = 0.0;
        sensor_msgs::msg::Imu last_valid_imu;
        last_valid_imu.orientation.w = 1.0;
        
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> last_pose_msg = nullptr;
        
        if (!reader_ || !reader_->has_next()) {
            RCLCPP_WARN(this->get_logger(), "Reader not initialized or bag is empty");
            return;
        }
        
        while (reader_->has_next()) {
            auto msg = reader_->read_next();
        
            // Deserialize based on topic name
            if (msg->topic_name == "/camera/image") {
                frame = FrameData();
                auto image_msg = deserializeMessage<sensor_msgs::msg::Image>(msg);

                // Convert ROS image to OpenCV Mat
                cv::Mat current_mat;
                auto cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
                current_mat = cv_ptr->image;

                // 1. Feature Detection and Description
                std::vector<cv::KeyPoint> kp;
                cv::Mat des;
                fe_method->detectAndCompute(current_mat, cv::Mat(), kp, des);

                if (path_data_.empty() || compare_features(des, path_data_.back().features.descriptors) < similarity_threshold) {
                    frame.features = Features();
                    frame.features.keypoints = kp;
                    frame.features.descriptors = des;

                    // Assign averaged IMU data
                    if (imu_count > 0) {
                        frame.imu.linear_acceleration.x = acc_x_sum / imu_count;
                        frame.imu.linear_acceleration.y = acc_y_sum / imu_count;
                        frame.imu.linear_acceleration.z = acc_z_sum / imu_count;
                        
                        frame.imu.angular_velocity.x = gyro_x_sum / imu_count;
                        frame.imu.angular_velocity.y = gyro_y_sum / imu_count;
                        frame.imu.angular_velocity.z = gyro_z_sum / imu_count;
                        
                        // For orientation, we use the latest one as averaging quaternions is complex
                        frame.imu.orientation = last_orientation;
                        
                        // Update last valid IMU
                        last_valid_imu = frame.imu;
                    } else {
                        // Use last valid IMU data if no new data
                        frame.imu = last_valid_imu;
                    }

                    // Assign averaged Altitude
                    if (alt_count > 0) {
                        last_valid_altitude = alt_sum / alt_count;
                    }
                    frame.altitude.data = last_valid_altitude;

                    if (last_pose_msg) {
                    // Create a SerializedMessage object from the bag data
                        auto pose_msg = deserializeMessage<geometry_msgs::msg::PoseArray>(last_pose_msg);

                        frame.target_pose = pose_msg->poses[2];
                    }
                    path_data_.push_back(frame);

                    // Reset accumulators
                    acc_x_sum = 0; acc_y_sum = 0; acc_z_sum = 0;
                    gyro_x_sum = 0; gyro_y_sum = 0; gyro_z_sum = 0;
                    alt_sum = 0;
                    imu_count = 0;
                    alt_count = 0;

                    cv::imshow("Image", current_mat);
                    cv::waitKey(1);
                }
            }
            else if (msg->topic_name == "/mavros/imu/data") {
                auto imu_msg = deserializeMessage<sensor_msgs::msg::Imu>(msg);
                
                acc_x_sum += imu_msg->linear_acceleration.x;
                acc_y_sum += imu_msg->linear_acceleration.y;
                acc_z_sum += imu_msg->linear_acceleration.z;
                
                gyro_x_sum += imu_msg->angular_velocity.x;
                gyro_y_sum += imu_msg->angular_velocity.y;
                gyro_z_sum += imu_msg->angular_velocity.z;
                
                last_orientation = imu_msg->orientation;
                imu_count++;
            }
            else if (msg->topic_name == "/mavros/global_position/rel_alt") {
                auto alt_msg = deserializeMessage<std_msgs::msg::Float64>(msg);
                alt_sum += alt_msg->data;
                alt_count++;
            }
            else if (msg->topic_name == "/simulation_pose_info") {
                last_pose_msg = msg;
            }
            if (topic_name == "/camera/camera_info") {
                if (vo_calculator.K_received_) {
                continue; // Already received K, skip
                }
                // Deserialize the last message into a CameraInfo message
                auto camera_info_msg = deserializeMessage<sensor_msgs::msg::CameraInfo>(data.last_message);
                vo_calculator.set_K_from_CameraInfo(camera_info_msg);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Created path with %zu frames", path_data_.size());
    }

    double compare_features(const cv::Mat& des1 , const cv::Mat& des2)
    {
        if (des1.rows < MIN_DESCRIPTORS_FOR_MATCHING || des2.rows < MIN_DESCRIPTORS_FOR_MATCHING)
        {
            // std::cerr << "Warning: No descriptors found or not enough for knnMatch." << std::endl;
            return 0.0;
        }
        
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(des1, des2, matches, MIN_DESCRIPTORS_FOR_MATCHING); // k=2 for ratio test

        // 3. Ratio Test (Lowe's ratio test)
        std::vector<cv::DMatch> good_matches;
        for (const auto& match_pair : matches)
        {
            if (match_pair.size() == MIN_DESCRIPTORS_FOR_MATCHING && 
                match_pair[0].distance < RATIO_TEST_THRESHOLD * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        if (good_matches.size() < MIN_GOOD_MATCHES)
        {
            // std::cerr << "Warning: Not enough good matches: " << good_matches.size() << std::endl;
            return 0.0;
        }

        double match_size = static_cast<double>(good_matches.size());

        return match_size / (double)des1.rows;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreatePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}