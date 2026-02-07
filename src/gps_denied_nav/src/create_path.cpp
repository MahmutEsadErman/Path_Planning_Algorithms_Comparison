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
    bool K_received_;
    cv::Mat K_;

    // comparing image similarity
    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Parameters
    int similarity_threshold;
    bool debug;
    double starting_second_;
    
    // Feature matching constants
    static constexpr float RATIO_TEST_THRESHOLD = 0.7f;
    static constexpr float ratio_test_k = 0.7f;
    
    // For debug visualization
    cv::Ptr<cv::DescriptorMatcher> flann_;
    cv::Mat cam_tf;
    cv::Mat visual_T;
    cv::Mat gt_T;
    cv::Mat img_matches;
    size_t match_size;
    double camera_pitch_angle=90.0;
    
    // Previous data for T calculation
    Features prev_features_;
    cv::Mat prev_frame_;
    geometry_msgs::msg::PoseArray::SharedPtr prev_pose_msg_;

    CreatePathNode() : Node("create_path_node"), timer_started_(false)
    {   
        this->declare_parameter<std::string>("feature_detector", "SURF");
        this->declare_parameter<std::string>("bag_file_path", "path_90degree");
        this->declare_parameter<std::string>("output_file", "");
        this->declare_parameter<int>("similarity_threshold", 60);
        this->declare_parameter<double>("starting_second", 1.0);
        this->declare_parameter<bool>("debug", false);
        similarity_threshold = this->get_parameter("similarity_threshold").as_int();
        debug = this->get_parameter("debug").as_bool();
        starting_second_ = this->get_parameter("starting_second").as_double();

        K_received_ = false;

        // Initialize feature detector and FLANN matcher
        std::string feature_detector = this->get_parameter("feature_detector").as_string();

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
        
        // Initialize flann_ for debug T calculation
        flann_ = matcher;
        
        // Step 1: Define C_Cros_Ccv (OpenCV Cam to ROS-style Cam)
        // OpenCV (Ccv): X-right, Y-down, Z-forward
        // ROS-style (Cros): X-forward, Y-left, Z-up
        cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,   // ROS X = CV Z
            -1,  0,  0,   // ROS Y = -CV X
             0, -1,  0);  // ROS Z = -CV Y

        // Step 2: Define C_B_Cros (ROS-style Cam to Drone Body)
        // This is the static camera pitch angle around the Y-axis.
        double angle_rad = camera_pitch_angle * M_PI / 180.0;
        cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
            cos(angle_rad), 0, sin(angle_rad),
                         0, 1,              0,
           -sin(angle_rad), 0, cos(angle_rad)
        );
        
        // Step 3: Combine them to get C_B_Ccv (OpenCV Cam to Drone Body)
        cam_tf = C_B_Cros * C_Cros_Ccv;
        
        match_size = 0;

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

        // Determine output file name
        std::string output_file;
        if (this->get_parameter("output_file").as_string() == "") {
            output_file = this->get_parameter("bag_file_path").as_string() + "_" + feature_detector + ".yaml";
        } else {
            output_file = this->get_parameter("output_file").as_string() + "_" + feature_detector + ".yaml";
        }
        save_path(output_file);

        exit(0);
    }

    void save_path(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
            return;
        }

        fs << "K" << K_;

        fs << "feature_detector" << this->get_parameter("feature_detector").as_string();

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
        
        // Track the first timestamp for relative time calculation
        int64_t first_timestamp = -1;
        
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

                // Get timestamp from the image message header
                int64_t current_timestamp = rclcpp::Time(image_msg->header.stamp).nanoseconds();
                
                // Capture the first timestamp to calculate relative time
                if (first_timestamp < 0) {
                    first_timestamp = current_timestamp;
                }
                
                // Skip frames before starting_second_ (relative to bag start)
                double relative_time = (current_timestamp - first_timestamp) / 1e9;
                if (relative_time < starting_second_) {
                    continue;
                }

                // Convert ROS image to OpenCV Mat
                cv::Mat current_mat;
                auto cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
                current_mat = cv_ptr->image;

                // 1. Feature Detection and Description
                std::vector<cv::KeyPoint> kp;
                cv::Mat des;
                fe_method->detectAndCompute(current_mat, cv::Mat(), kp, des);

                // if kp size is less than similarity threshold, skip this frame because there are not enough features
                if (kp.size() < similarity_threshold+10) {
                    continue;
                }

                if (path_data_.empty() || compare_features(des, path_data_.back().features.descriptors)) {
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
                    imu_count = 0;

                    if (debug) {
                        std::cout << "kp size: " << kp.size() << std::endl;
                        cv::imshow("Image", current_mat);
                        
                        // Calculate and display T if we have previous frames
                        if (!prev_features_.descriptors.empty() && prev_pose_msg_) {
                            // Calculate visual odometry T using pre-computed features
                            calculate_T_with_frames(frame.features, prev_features_, current_mat, prev_frame_);
                            
                            // Calculate ground truth T
                            if (last_pose_msg) {
                                auto current_pose = deserializeMessage<geometry_msgs::msg::PoseArray>(last_pose_msg);
                                calculate_gt_T(prev_pose_msg_->poses[2], current_pose->poses[2]);
                            }
                            
                            visualize_matches();
                        }
                        
                        // Store current data for next iteration
                        prev_features_ = frame.features;
                        prev_frame_ = current_mat.clone();
                        if (last_pose_msg) {
                            prev_pose_msg_ = deserializeMessage<geometry_msgs::msg::PoseArray>(last_pose_msg);
                        }
                        
                        int key = cv::waitKey(0);
                        if (key == 'q' || key == 'Q') {
                            std::cout << "User pressed 'q', exiting..." << std::endl;
                            cv::destroyAllWindows();
                            exit(0);
                        }
                    }
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
                last_valid_altitude = alt_msg->data;
            }
            else if (msg->topic_name == "/simulation_pose_info") {
                last_pose_msg = msg;
            }
            if (msg->topic_name == "/camera/camera_info") {
                if (K_received_) {
                continue; // Already received K, skip
                }
                // Deserialize the last message into a CameraInfo message
                auto camera_info_msg = deserializeMessage<sensor_msgs::msg::CameraInfo>(msg);
                set_K_from_CameraInfo(camera_info_msg);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Created path with %zu frames", path_data_.size());
    }

    bool compare_features(const cv::Mat& des1 , const cv::Mat& des2)
    {   
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(des1, des2, matches, 2); // k=2 for ratio test

        // 3. Ratio Test (Lowe's ratio test)
        std::vector<cv::DMatch> good_matches;
        for (const auto& match_pair : matches)
        {
            // Ensure we have at least 2 matches for ratio test
            if (match_pair.size() < 2) {
                continue;
            }
            if (match_pair[0].distance < RATIO_TEST_THRESHOLD * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        if (good_matches.size() > similarity_threshold) {
            return false;
        }

        return true;
    }

    // This part is added for debug purposes

    void visualize_matches()
    {
        if (K_received_)
        {   
            if (match_size < 10)
            {
                std::cerr << "Warning: Not enough good matches: " << match_size << std::endl;
                return;
            }
            compare_transformations();
            cv::imshow("matches", img_matches);
            cv::waitKey(1);
        }
        else
        {
            std::cerr << "Warning: Camera intrinsics not set." << std::endl;
        }
    }

    void calculate_T_with_frames(const Features& features1, const Features& features2,
                                    const cv::Mat& frame1, const cv::Mat& frame2)
    {
        if (!K_received_)
        {
            std::cerr << "Warning: Waiting for camera intrinsics..." << std::endl;
            return;
        }

        // Use pre-computed features instead of detecting again
        const std::vector<cv::KeyPoint>& kp1 = features1.keypoints;
        const std::vector<cv::KeyPoint>& kp2 = features2.keypoints;
        const cv::Mat& des1 = features1.descriptors;
        const cv::Mat& des2 = features2.descriptors;

        if (des1.empty() || des2.empty())
        {
            std::cerr << "Warning: No descriptors found." << std::endl;
            return;
        }
        
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        flann_->knnMatch(des1, des2, matches, 2); // k=2 for ratio test

        // 3. Ratio Test
        std::vector<cv::DMatch> good_matches;
        for (const auto& match_pair : matches)
        {
            if (match_pair.size() == 2 && match_pair[0].distance < ratio_test_k * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        if (good_matches.size() < 10)
        {
            std::cerr << "Warning: Not enough good matches: " << good_matches.size() << std::endl;
            return;
        }

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

        // 6. Transform from OpenCV Camera Frame to Drone Body Frame

        // Transform rotation: R_body = C * R_cam * C^T
        cv::Mat R_ros = cam_tf * R * cam_tf.t(); // C.t() is C-transpose (which is C-inverse for rotation)

        // Transform translation: t_body = C * t_cam
        cv::Mat t_ros = cam_tf * t;

        // 7. Integrate motion
        visual_T = cv::Mat::eye(4, 4, CV_64F);
        R_ros.copyTo(visual_T(cv::Rect(0, 0, 3, 3))); // Copy transformed R to T's rotation part
        t_ros.copyTo(visual_T(cv::Rect(3, 0, 1, 3))); // Copy transformed t to T's translation part

        cv::drawMatches(frame1, kp1, frame2, kp2, good_matches, img_matches);
        match_size = good_matches.size();

        // Calculate target angle from translation vector
        // atan2(y, x) where y=Y_component (left), x=X_component (forward)
        double target_angle = std::atan2(t_ros.at<double>(1), t_ros.at<double>(0));
        std::cout << "Target angle: " << target_angle * 180.0 / M_PI << " degrees" << std::endl;
    }

    void calculate_gt_T(const geometry_msgs::msg::Pose pose_msg1,
                             const geometry_msgs::msg::Pose pose_msg2)
    {   
        // Helper lambda to convert quaternion to rotation matrix
        auto quaternionToRotationMatrix = [](double x, double y, double z, double w) -> cv::Mat {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            
            // Normalize quaternion
            double norm = std::sqrt(x*x + y*y + z*z + w*w);
            x /= norm; y /= norm; z /= norm; w /= norm;
            
            // Rotation matrix from quaternion
            R.at<double>(0, 0) = 1 - 2*(y*y + z*z);
            R.at<double>(0, 1) = 2*(x*y - z*w);
            R.at<double>(0, 2) = 2*(x*z + y*w);
            
            R.at<double>(1, 0) = 2*(x*y + z*w);
            R.at<double>(1, 1) = 1 - 2*(x*x + z*z);
            R.at<double>(1, 2) = 2*(y*z - x*w);
            
            R.at<double>(2, 0) = 2*(x*z - y*w);
            R.at<double>(2, 1) = 2*(y*z + x*w);
            R.at<double>(2, 2) = 1 - 2*(x*x + y*y);
            
            return R;
        };
        
        // Convert pose1 to transformation matrix
        cv::Mat T1 = cv::Mat::eye(4, 4, CV_64F);
        
        // Extract rotation from quaternion
        cv::Mat R1 = quaternionToRotationMatrix(
            pose_msg1.orientation.x,
            pose_msg1.orientation.y,
            pose_msg1.orientation.z,
            pose_msg1.orientation.w
        );
        R1.copyTo(T1(cv::Rect(0, 0, 3, 3)));
        
        // Extract translation
        T1.at<double>(0, 3) = pose_msg1.position.x;
        T1.at<double>(1, 3) = pose_msg1.position.y;
        T1.at<double>(2, 3) = pose_msg1.position.z;

        // Convert pose2 to transformation matrix
        cv::Mat T2 = cv::Mat::eye(4, 4, CV_64F);
        
        // Extract rotation from quaternion
        cv::Mat R2 = quaternionToRotationMatrix(
            pose_msg2.orientation.x,
            pose_msg2.orientation.y,
            pose_msg2.orientation.z,
            pose_msg2.orientation.w
        );
        R2.copyTo(T2(cv::Rect(0, 0, 3, 3)));
        
        // Extract translation
        T2.at<double>(0, 3) = pose_msg2.position.x;
        T2.at<double>(1, 3) = pose_msg2.position.y;
        T2.at<double>(2, 3) = pose_msg2.position.z;

        // Compute relative transformation: T_relative = T1^-1 * T2
        gt_T = T1.inv() * T2;
    }

    void compare_transformations()
    {
        if (visual_T.empty() || gt_T.empty())
        {
            std::cerr << "Warning: Cannot compare - transformations not computed yet." << std::endl;
            return;
        }

        // Extract rotation matrices
        cv::Mat R_visual = visual_T(cv::Rect(0, 0, 3, 3));
        cv::Mat R_gt = gt_T(cv::Rect(0, 0, 3, 3));
        
        // Extract translation vectors
        cv::Mat t_visual = visual_T(cv::Rect(3, 0, 1, 3));
        cv::Mat t_gt = gt_T(cv::Rect(3, 0, 1, 3));

        double gt_magnitude = cv::norm(t_gt);
        double visual_magnitude = cv::norm(t_visual);
        t_visual = t_visual * (gt_magnitude / visual_magnitude);

        // 1. Calculate rotation error (angle difference in degrees)
        cv::Mat R_error = R_gt.t() * R_visual; // R_gt^T * R_visual
        double trace = R_error.at<double>(0,0) + R_error.at<double>(1,1) + R_error.at<double>(2,2);
        double rotation_error_rad = std::acos(std::min(1.0, std::max(-1.0, (trace - 1.0) / 2.0)));
        double rotation_error_deg = rotation_error_rad * 180.0 / M_PI;

        // 2. Calculate translation direction error (angular difference)
        cv::Mat t_visual_norm = t_visual / cv::norm(t_visual);
        cv::Mat t_gt_norm = t_gt / cv::norm(t_gt);
        double dot_product = t_visual_norm.dot(t_gt_norm);
        double translation_angle_error_rad = std::acos(std::min(1.0, std::max(-1.0, dot_product)));
        double translation_angle_error_deg = translation_angle_error_rad * 180.0 / M_PI;

        // 3. Calculate translation magnitude error
        double t_visual_magnitude = cv::norm(t_visual);
        double t_gt_magnitude = cv::norm(t_gt);
        double translation_magnitude_error = std::abs(t_visual_magnitude - t_gt_magnitude);
        double translation_magnitude_error_percent = (translation_magnitude_error / t_gt_magnitude) * 100.0;

        // 4. Calculate Euclidean distance between translation vectors
        double translation_euclidean_error = cv::norm(t_visual - t_gt);

        // Print comparison results
        std::cout << "\n========== Transformation Comparison ==========" << std::endl;
        std::cout << "Rotation Error: " << rotation_error_deg << " degrees" << std::endl;
        std::cout << "\nTranslation:" << std::endl;
        std::cout << "  Visual Odometry: [" << t_visual.at<double>(0) << ", " 
                  << t_visual.at<double>(1) << ", " << t_visual.at<double>(2) << "]" << std::endl;
        std::cout << "  Ground Truth:    [" << t_gt.at<double>(0) << ", " 
                  << t_gt.at<double>(1) << ", " << t_gt.at<double>(2) << "]" << std::endl;
        std::cout << "  Visual Magnitude: " << t_visual_magnitude << std::endl;
        std::cout << "  GT Magnitude:     " << t_gt_magnitude << std::endl;
        std::cout << "  Direction Error:  " << translation_angle_error_deg << " degrees" << std::endl;
        std::cout << "  Magnitude Error:  " << translation_magnitude_error 
                  << " (" << translation_magnitude_error_percent << "%)" << std::endl;
        std::cout << "  Euclidean Error:  " << translation_euclidean_error << std::endl;
        std::cout << "  Number of Matches Used: " << match_size << std::endl;
        std::cout << "=============================================\n" << std::endl;
    }

    

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreatePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}