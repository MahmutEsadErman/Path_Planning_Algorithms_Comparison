/**
 * @file test_yaw_methods.cpp
 * @brief Test and compare different yaw calculation methods:
 *        1. Essential Matrix
 *        2. Homography
 *        3. Average Displacement
 *        4. Median Displacement
 *        5. RANSAC Displacement
 *        6. Hybrid EM+Avg
 *        7. Direction Cue (features relative to frame center)
 * 
 * Usage: ros2 run gps_denied_nav test_yaw_methods --ros-args -p path_file:=path.yaml
 */

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <numeric>
#include <iomanip>
#include <algorithm>

struct Features {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct FrameData {
    Features features;
    double altitude;
    struct {
        double x, y, z;
    } position;
    struct {
        double x, y, z, w;
    } orientation;
};

struct MethodResult {
    std::string name;
    double yaw_rad;
    double time_us;
    bool valid;
};

struct ComparisonResult {
    int frame_pair;
    int num_matches;
    double ground_truth_yaw;           // GT for relative displacement methods
    double ground_truth_direction;     // GT for direction cue method
    MethodResult essential_matrix;
    MethodResult homography;
    MethodResult avg_displacement;
    MethodResult median_displacement;
    MethodResult ransac_displacement;
    MethodResult hybrid_em_avg;
    MethodResult direction_cue;
    
    // Debug info
    double drone_yaw;         // Drone heading at frame1
    double dx_global;         // Global X displacement
    double dy_global;         // Global Y displacement
    double dx_body;           // Body X displacement (forward)
    double dy_body;           // Body Y displacement (left)
};

class YawMethodTester : public rclcpp::Node {
public:
    YawMethodTester() : Node("test_yaw_methods") {
        // Parameters
        this->declare_parameter<std::string>("path_file", "yeni_harita_SURF.yaml");
        this->declare_parameter<double>("camera_pitch_angle", 90.0);
        this->declare_parameter<int>("iterations", 100);
        this->declare_parameter<int>("max_pairs", 50);
        this->declare_parameter<int>("skip_frames", 10);
        
        path_file_ = this->get_parameter("path_file").as_string();
        camera_pitch_angle_ = this->get_parameter("camera_pitch_angle").as_double();
        iterations_ = this->get_parameter("iterations").as_int();
        max_pairs_ = this->get_parameter("max_pairs").as_int();
        skip_frames_ = this->get_parameter("skip_frames").as_int();
        
        // Initialize camera transform
        initCameraTransform();
        
        // Load path data
        if (!loadPath(path_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path file: %s", path_file_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu frames from %s", frames_.size(), path_file_.c_str());
        
        // Initialize matcher
        if (feature_detector_ == "ORB") {
            matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        } else {
            matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::KDTreeIndexParams>(5));
        }
        
        // Run tests
        runTests();
        
        // Print results
        printResults();
    }

private:
    std::string path_file_;
    std::string feature_detector_;
    double camera_pitch_angle_;
    int iterations_;
    int max_pairs_;
    int skip_frames_;
    
    cv::Mat K_;
    cv::Mat cam_tf_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    
    std::vector<FrameData> frames_;
    std::vector<ComparisonResult> results_;
    
    void initCameraTransform() {
        // OpenCV Cam to ROS-style Cam
        cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,
            -1,  0,  0,
             0, -1,  0);

        // ROS-style Cam to Drone Body (pitch rotation)
        double angle_rad = camera_pitch_angle_ * M_PI / 180.0;
        cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
            cos(angle_rad), 0, sin(angle_rad),
                         0, 1,              0,
           -sin(angle_rad), 0, cos(angle_rad));

        cam_tf_ = C_B_Cros * C_Cros_Ccv;
    }
    
    bool loadPath(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }

        fs["K"] >> K_;
        fs["feature_detector"] >> feature_detector_;

        cv::FileNode frames_node = fs["frames"];
        if (frames_node.type() != cv::FileNode::SEQ) {
            return false;
        }

        for (auto it = frames_node.begin(); it != frames_node.end(); ++it) {
            cv::FileNode frame_node = *it;
            FrameData frame;

            // Skip low altitude frames
            double altitude = (double)frame_node["altitude"];
            if (altitude < 49) continue;
            
            frame.altitude = altitude;

            // Keypoints and Descriptors
            frame_node["keypoints"] >> frame.features.keypoints;
            frame_node["descriptors"] >> frame.features.descriptors;

            // Target Pose
            if (!frame_node["target_pose"].empty()) {
                cv::FileNode pos_node = frame_node["target_pose"]["position"];
                frame.position.x = (double)pos_node["x"];
                frame.position.y = (double)pos_node["y"];
                frame.position.z = (double)pos_node["z"];
                
                cv::FileNode orient_node = frame_node["target_pose"]["orientation"];
                frame.orientation.x = (double)orient_node["x"];
                frame.orientation.y = (double)orient_node["y"];
                frame.orientation.z = (double)orient_node["z"];
                frame.orientation.w = (double)orient_node["w"];
            }

            frames_.push_back(frame);
        }

        fs.release();
        return !frames_.empty();
    }
    
    double quaternionToYaw(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    struct GTResult {
        double relative_yaw;
        double drone_yaw;
        double dx_global;
        double dy_global;
        double dx_body;
        double dy_body;
    };
    
    GTResult calculateGroundTruthYaw(const FrameData& frame1, const FrameData& frame2) {
        GTResult result;
        
        // 1. Get drone's current heading (yaw)
        result.drone_yaw = quaternionToYaw(
            frame1.orientation.x, frame1.orientation.y,
            frame1.orientation.z, frame1.orientation.w);
        
        // 2. Calculate global displacement
        result.dx_global = frame2.position.x - frame1.position.x;
        result.dy_global = frame2.position.y - frame1.position.y;
        
        // 3. Calculate Global Motion Direction
        double global_motion_yaw = std::atan2(result.dy_global, result.dx_global);
        
        // 4. Calculate Relative Motion Yaw (Body Frame)
        // This is the direction of motion relative to where the drone is facing
        result.relative_yaw = global_motion_yaw - result.drone_yaw;
        
        // Normalize to [-pi, pi]
        while (result.relative_yaw > M_PI) result.relative_yaw -= 2 * M_PI;
        while (result.relative_yaw < -M_PI) result.relative_yaw += 2 * M_PI;
        
        // For debug, we can still compute body dx/dy using simple rotation logic if needed,
        // but for now we'll imply them from relative yaw and magnitude.
        double magnitude = std::sqrt(result.dx_global*result.dx_global + result.dy_global*result.dy_global);
        result.dx_body = magnitude * std::cos(result.relative_yaw);
        result.dy_body = magnitude * std::sin(result.relative_yaw);
        
        return result;
    }
    
    // Ground truth for direction cue: angle from current drone to the NEXT waypoint
    // This is what direction cue measures - "where is the target relative to me?"
    double calculateDirectionGroundTruth(const FrameData& current_frame, const FrameData& target_frame) {
        double drone_yaw = quaternionToYaw(
            current_frame.orientation.x, current_frame.orientation.y,
            current_frame.orientation.z, current_frame.orientation.w);
        
        // Global angle from current position to target position
        double dx = target_frame.position.x - current_frame.position.x;
        double dy = target_frame.position.y - current_frame.position.y;
        double global_angle_to_target = std::atan2(dy, dx);
        
        // Relative angle: target direction minus drone heading
        double relative_angle = global_angle_to_target - drone_yaw;
        
        // Normalize to [-pi, pi]
        while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
        while (relative_angle < -M_PI) relative_angle += 2 * M_PI;
        
        return relative_angle;
    }
    
    std::vector<cv::DMatch> matchFeatures(const cv::Mat& des1, const cv::Mat& des2) {
        std::vector<cv::DMatch> good_matches;
        if (des1.rows < 2 || des2.rows < 2) return good_matches;
        
        std::vector<std::vector<cv::DMatch>> matches;
        matcher_->knnMatch(des1, des2, matches, 2);
        
        for (const auto& m : matches) {
            if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
                good_matches.push_back(m[0]);
            }
        }
        return good_matches;
    }
    
    // ==================== METHOD 1: Essential Matrix ====================
    MethodResult essentialMatrixMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Essential Matrix";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.size() < 10) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            cv::Mat E, R, t, mask;
            E = cv::findEssentialMat(pts1, pts2, K_, cv::USAC_MAGSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, pts1, pts2, K_, R, t, mask);
            
            // NOTE: recoverPose returns 't' such that x2 = R*x1 + t
            // If camera moves forward, points move backward relative to camera.
            // So 't' represents the motion of the scene, which is opposite to camera motion.
            // We interpret (-t) as the camera velocity vector direction.
            cv::Mat t_body = cam_tf_ * (-t);
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 2: Homography ====================
    MethodResult homographyMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Homography";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.size() < 10) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            cv::Mat H = cv::findHomography(pts1, pts2, cv::USAC_MAGSAC);
            
            if (H.empty()) continue;
            
            // Calculate simple average flow for guidance (robust reference)
            cv::Point2f avg_flow(0, 0);
            for(size_t k=0; k<pts1.size(); ++k) {
                avg_flow += (pts2[k] - pts1[k]);
            }
            if(!pts1.empty()) avg_flow /= (float)pts1.size();
            // Expected cam movement direction is opposite to flow
            cv::Vec2f guide_dir(-avg_flow.x, -avg_flow.y); 
            
            // Decompose homography assuming planar scene
            std::vector<cv::Mat> Rs, ts, normals;
            int solutions = cv::decomposeHomographyMat(H, K_, Rs, ts, normals);
            
            int best_idx = -1;
            double max_dot = -1e9;
            
            for (int s = 0; s < solutions; ++s) {
                // Get translation in image plane (x, y)
                // Note: ts[s] is 3x1 vector. 
                // We compare the direction of translation t with the guide direction.
                // t represents camera translation.
                
                // Project t to image plane to compare with flow
                // Flow is in pixel space, t is in normalized camera space.
                // But the direction (angle) should be roughly correlated.
                // t.x is aligned with cols, t.y with rows.
                
                float tx = (float)ts[s].at<double>(0);
                float ty = (float)ts[s].at<double>(1);
                
                // Use dot product to find best alignment with guide direction
                // We use (-t) because previously we determined we need to invert t.
                // Let's check alignment of (-t) with (-flow).
                // Simplified: compare (tx, ty) with (guide_dir) ? 
                // Let's stick to the convention we found working:
                // Camera Frame Motion ~ (-tx, -ty)
                // Guide (Avg Displacement) ~ (-flow.x, -flow.y)
                
                // Actually, avg_flow in pixels is roughly K * t (ignoring depth/rotation).
                // So avg_flow direction should be roughly opposite to t direction?
                // Let's rely on the previous finding: Essential Matrix worked with (-t).
                // Avg Displacement logic used (-dx, -dy).
                // So we want the solution where (-ts[s]) is aligned with (-avg_flow).
                
                float cam_dx = -tx;
                float cam_dy = -ty;
                
                float dot = cam_dx * guide_dir[0] + cam_dy * guide_dir[1];
                
                if (dot > max_dot) {
                    max_dot = dot;
                    best_idx = s;
                }
            }
            
            if (best_idx >= 0) {
                // Use (-t) as camera motion vector
                cv::Mat t_body = cam_tf_ * (-ts[best_idx]);
                result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 3: Average Displacement ====================
    MethodResult avgDisplacementMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Avg Displacement";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.empty()) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            double sum_dx = 0.0, sum_dy = 0.0;
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                sum_dx += pts2_norm[j].x - pts1_norm[j].x;
                sum_dy += pts2_norm[j].y - pts1_norm[j].y;
            }
            
            double avg_dx = sum_dx / pts1_norm.size();
            double avg_dy = sum_dy / pts1_norm.size();
            
            cv::Mat t_cam = (cv::Mat_<double>(3, 1) << -avg_dx, -avg_dy, 1.0);
            cv::Mat t_body = cam_tf_ * t_cam;
            
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 4: Median Displacement ====================
    // Uses median of per-match angles instead of mean for outlier robustness
    MethodResult medianDisplacementMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Median Displacement";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.empty()) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            // Compute per-match displacement vectors
            std::vector<double> dx_vals, dy_vals;
            dx_vals.reserve(pts1_norm.size());
            dy_vals.reserve(pts1_norm.size());
            
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                dx_vals.push_back(pts2_norm[j].x - pts1_norm[j].x);
                dy_vals.push_back(pts2_norm[j].y - pts1_norm[j].y);
            }
            
            // Find median of dx and dy separately
            std::sort(dx_vals.begin(), dx_vals.end());
            std::sort(dy_vals.begin(), dy_vals.end());
            
            double median_dx = dx_vals[dx_vals.size() / 2];
            double median_dy = dy_vals[dy_vals.size() / 2];
            
            cv::Mat t_cam = (cv::Mat_<double>(3, 1) << -median_dx, -median_dy, 1.0);
            cv::Mat t_body = cam_tf_ * t_cam;
            
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 5: RANSAC-filtered Displacement ====================
    // Rejects outlier matches via iterative sigma-clipping before averaging
    MethodResult ransacDisplacementMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "RANSAC Displacement";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.empty()) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            // Step 1: Compute all displacement vectors
            std::vector<double> dx_all, dy_all;
            dx_all.reserve(pts1_norm.size());
            dy_all.reserve(pts1_norm.size());
            
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                dx_all.push_back(pts2_norm[j].x - pts1_norm[j].x);
                dy_all.push_back(pts2_norm[j].y - pts1_norm[j].y);
            }
            
            // Step 2: Compute median as robust initial estimate
            std::vector<double> dx_sorted = dx_all, dy_sorted = dy_all;
            std::sort(dx_sorted.begin(), dx_sorted.end());
            std::sort(dy_sorted.begin(), dy_sorted.end());
            double center_dx = dx_sorted[dx_sorted.size() / 2];
            double center_dy = dy_sorted[dy_sorted.size() / 2];
            
            // Step 3: Compute distances from median and find MAD (median absolute deviation)
            std::vector<double> distances;
            distances.reserve(dx_all.size());
            for (size_t j = 0; j < dx_all.size(); ++j) {
                double ddx = dx_all[j] - center_dx;
                double ddy = dy_all[j] - center_dy;
                distances.push_back(std::sqrt(ddx*ddx + ddy*ddy));
            }
            std::vector<double> dist_sorted = distances;
            std::sort(dist_sorted.begin(), dist_sorted.end());
            double mad = dist_sorted[dist_sorted.size() / 2];
            double threshold = std::max(mad * 2.5, 0.001); // 2.5 * MAD, with minimum
            
            // Step 4: Keep only inliers and recompute mean
            double sum_dx = 0.0, sum_dy = 0.0;
            int inlier_count = 0;
            for (size_t j = 0; j < dx_all.size(); ++j) {
                if (distances[j] <= threshold) {
                    sum_dx += dx_all[j];
                    sum_dy += dy_all[j];
                    inlier_count++;
                }
            }
            
            if (inlier_count == 0) {
                // Fallback to median
                sum_dx = center_dx;
                sum_dy = center_dy;
                inlier_count = 1;
            }
            
            double avg_dx = sum_dx / inlier_count;
            double avg_dy = sum_dy / inlier_count;
            
            cv::Mat t_cam = (cv::Mat_<double>(3, 1) << -avg_dx, -avg_dy, 1.0);
            cv::Mat t_body = cam_tf_ * t_cam;
            
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 6: Hybrid EM + Avg Displacement ====================
    // Uses EM as primary but AvgDisplacement as sanity check to catch 180° flips
    MethodResult hybridMethod(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Hybrid EM+Avg";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.size() < 10) return result;
        
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            // --- Fast Avg Displacement as reference ---
            std::vector<cv::Point2f> pts1_norm, pts2_norm;
            cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
            cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
            
            double sum_dx = 0.0, sum_dy = 0.0;
            for (size_t j = 0; j < pts1_norm.size(); ++j) {
                sum_dx += pts2_norm[j].x - pts1_norm[j].x;
                sum_dy += pts2_norm[j].y - pts1_norm[j].y;
            }
            double avg_dx = sum_dx / pts1_norm.size();
            double avg_dy = sum_dy / pts1_norm.size();
            
            cv::Mat t_cam_avg = (cv::Mat_<double>(3, 1) << -avg_dx, -avg_dy, 1.0);
            cv::Mat t_body_avg = cam_tf_ * t_cam_avg;
            double avg_yaw = std::atan2(t_body_avg.at<double>(1), t_body_avg.at<double>(0));
            
            // --- Essential Matrix ---
            cv::Mat E, R, t, mask;
            E = cv::findEssentialMat(pts1, pts2, K_, cv::USAC_MAGSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, pts1, pts2, K_, R, t, mask);
            
            cv::Mat t_body_em = cam_tf_ * (-t);
            double em_yaw = std::atan2(t_body_em.at<double>(1), t_body_em.at<double>(0));
            
            // --- Fusion logic ---
            double diff = std::abs(em_yaw - avg_yaw);
            // Normalize diff to [0, pi]
            while (diff > M_PI) diff -= 2 * M_PI;
            diff = std::abs(diff);
            
            if (diff < 0.52) {
                // <30°: EM and Avg agree → trust EM (more accurate)
                result.yaw_rad = em_yaw;
            } else if (diff > 2.62) {
                // >150°: EM is likely 180° flipped → invert EM
                result.yaw_rad = em_yaw + M_PI;
                // Normalize to [-pi, pi]
                while (result.yaw_rad > M_PI) result.yaw_rad -= 2 * M_PI;
                while (result.yaw_rad < -M_PI) result.yaw_rad += 2 * M_PI;
            } else {
                // Between 30°-150°: EM is unreliable → use Avg
                result.yaw_rad = avg_yaw;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    // ==================== METHOD 7: Direction Cue ====================
    // Instead of measuring displacement between frames, measures where
    // matched features appear in the CURRENT frame relative to center.
    // Features to the right of center → target is to the right → turn right
    MethodResult directionCueMethod(
        const std::vector<cv::KeyPoint>& kp_current,
        const std::vector<cv::DMatch>& matches)
    {
        MethodResult result;
        result.name = "Direction Cue";
        result.valid = false;
        result.yaw_rad = 0.0;
        result.time_us = 0.0;
        
        if (matches.empty()) return result;
        
        // Extract matched points from CURRENT camera frame (trainIdx)
        std::vector<cv::Point2f> pts_current;
        pts_current.reserve(matches.size());
        for (const auto& m : matches) {
            pts_current.push_back(kp_current[m.trainIdx].pt);
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations_; ++i) {
            // Normalize to camera coordinates (center becomes origin)
            std::vector<cv::Point2f> pts_norm;
            cv::undistortPoints(pts_current, pts_norm, K_, cv::noArray());
            
            // Each normalized point IS the direction from center
            std::vector<double> dx_vals, dy_vals;
            dx_vals.reserve(pts_norm.size());
            dy_vals.reserve(pts_norm.size());
            
            for (size_t j = 0; j < pts_norm.size(); ++j) {
                dx_vals.push_back(pts_norm[j].x);
                dy_vals.push_back(pts_norm[j].y);
            }
            
            // Use median for outlier robustness
            std::sort(dx_vals.begin(), dx_vals.end());
            std::sort(dy_vals.begin(), dy_vals.end());
            
            double median_dx = dx_vals[dx_vals.size() / 2];
            double median_dy = dy_vals[dy_vals.size() / 2];
            
            cv::Mat t_cam = (cv::Mat_<double>(3, 1) << median_dx, median_dy, 1.0);
            cv::Mat t_body = cam_tf_ * t_cam;
            
            result.yaw_rad = std::atan2(t_body.at<double>(1), t_body.at<double>(0));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / static_cast<double>(iterations_);
        result.valid = true;
        
        return result;
    }
    
    void runTests() {
        if (frames_.size() <= skip_frames_ + 1) {
            RCLCPP_WARN(this->get_logger(), "Not enough frames to run tests (Frames: %zu, Skip: %d)", 
                        frames_.size(), skip_frames_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Running tests on %d frame pairs (skipping first %d)...", 
                    std::min(max_pairs_, static_cast<int>(frames_.size()) - 1 - skip_frames_), skip_frames_);
        
        int pairs_tested = 0;
        
        for (size_t i = skip_frames_; i < frames_.size() - 1 && pairs_tested < max_pairs_; ++i) {
            auto& frame1 = frames_[i];
            auto& frame2 = frames_[i + 1];
            
            // Match features
            auto matches = matchFeatures(frame1.features.descriptors, 
                                         frame2.features.descriptors);
            
            if (matches.size() < 10) continue;
            
            ComparisonResult comp;
            comp.frame_pair = i;
            comp.num_matches = matches.size();
            
            // Calculate ground truth with debug info
            auto gt = calculateGroundTruthYaw(frame1, frame2);
            comp.ground_truth_yaw = gt.relative_yaw;
            comp.drone_yaw = gt.drone_yaw;
            comp.dx_global = gt.dx_global;
            comp.dy_global = gt.dy_global;
            comp.dx_body = gt.dx_body;
            comp.dy_body = gt.dy_body;
            
            // Ground truth for direction cue: angle to next waypoint
            comp.ground_truth_direction = calculateDirectionGroundTruth(frame1, frame2);
            
            // Test all methods
            comp.essential_matrix = essentialMatrixMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            comp.homography = homographyMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            comp.avg_displacement = avgDisplacementMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            comp.median_displacement = medianDisplacementMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            comp.ransac_displacement = ransacDisplacementMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            comp.hybrid_em_avg = hybridMethod(
                frame1.features.keypoints, frame2.features.keypoints, matches);
            
            // Direction cue uses only current frame keypoints (frame2 = current)
            comp.direction_cue = directionCueMethod(
                frame2.features.keypoints, matches);
            
            results_.push_back(comp);
            pairs_tested++;
            
            if (pairs_tested % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "Tested %d pairs...", pairs_tested);
            }
        }
    }
    
    void printResults() {
        if (results_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No results to display!");
            return;
        }
        
        // Calculate statistics
        struct Stats {
            std::vector<double> errors;
            std::vector<double> times;
            double mean_error = 0;
            double std_error = 0;
            double max_error = 0;
            double mean_time = 0;
        };
        
        Stats em_stats, hom_stats, avg_stats, med_stats, ransac_stats, hybrid_stats, dir_stats;
        
        for (const auto& r : results_) {
            double gt = r.ground_truth_yaw * 180.0 / M_PI;
            double gt_dir = r.ground_truth_direction * 180.0 / M_PI;
            
            auto addStats = [&](const MethodResult& mr, Stats& s, double ground_truth) {
                if (mr.valid) {
                    double err = std::abs(mr.yaw_rad * 180.0 / M_PI - ground_truth);
                    if (err > 180) err = 360 - err;
                    s.errors.push_back(err);
                    s.times.push_back(mr.time_us);
                }
            };
            
            addStats(r.essential_matrix, em_stats, gt);
            addStats(r.homography, hom_stats, gt);
            addStats(r.avg_displacement, avg_stats, gt);
            addStats(r.median_displacement, med_stats, gt);
            addStats(r.ransac_displacement, ransac_stats, gt);
            addStats(r.hybrid_em_avg, hybrid_stats, gt);
            addStats(r.direction_cue, dir_stats, gt_dir);  // Direction cue uses its own GT
        }
        
        // Calculate statistics
        auto calcStats = [](Stats& s) {
            if (s.errors.empty()) return;
            
            s.mean_error = std::accumulate(s.errors.begin(), s.errors.end(), 0.0) / s.errors.size();
            s.max_error = *std::max_element(s.errors.begin(), s.errors.end());
            s.mean_time = std::accumulate(s.times.begin(), s.times.end(), 0.0) / s.times.size();
            
            double sq_sum = 0;
            for (double e : s.errors) sq_sum += (e - s.mean_error) * (e - s.mean_error);
            s.std_error = std::sqrt(sq_sum / s.errors.size());
        };
        
        calcStats(em_stats);
        calcStats(hom_stats);
        calcStats(avg_stats);
        calcStats(med_stats);
        calcStats(ransac_stats);
        calcStats(hybrid_stats);
        calcStats(dir_stats);
        
        // Print results
        auto printMethodRow = [](const std::string& name, const Stats& s) {
            std::cout << "║  " << std::setw(22) << std::left << name 
                      << "  ║     " << std::setw(8) << s.mean_error 
                      << "  ║     " << std::setw(8) << s.std_error 
                      << "  ║      " << std::setw(8) << s.max_error << "   ║\n";
        };
        
        auto printSpeedRow = [](const std::string& name, const Stats& s, double baseline) {
            std::cout << "║  " << std::setw(22) << std::left << name 
                      << "  ║      " << std::setw(12) << s.mean_time 
                      << "     ║          " << std::setw(8) << (baseline / std::max(s.mean_time, 0.001)) << "x       ║\n";
        };
        
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                     YAW CALCULATION METHOD COMPARISON                        ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  Path File: " << std::setw(64) << std::left << path_file_ << "║\n";
        std::cout << "║  Feature Detector: " << std::setw(57) << std::left << feature_detector_ << "║\n";
        std::cout << "║  Frame Pairs Tested: " << std::setw(55) << std::left << results_.size() << "║\n";
        std::cout << "║  Iterations per Method: " << std::setw(52) << std::left << iterations_ << "║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║                              ACCURACY (degrees)                              ║\n";
        std::cout << "╠════════════════════════╦═══════════════╦═══════════════╦═══════════════════╣\n";
        std::cout << "║         Method         ║   Mean Error  ║   Std Dev     ║    Max Error      ║\n";
        std::cout << "╠════════════════════════╬═══════════════╬═══════════════╬═══════════════════╣\n";
        
        std::cout << std::fixed << std::setprecision(2);
        printMethodRow("Essential Matrix", em_stats);
        printMethodRow("Homography", hom_stats);
        printMethodRow("Avg Displacement", avg_stats);
        printMethodRow("Median Displacement", med_stats);
        printMethodRow("RANSAC Displacement", ransac_stats);
        printMethodRow("Hybrid EM+Avg", hybrid_stats);
        printMethodRow("Direction Cue *", dir_stats);
        
        std::cout << "╠════════════════════════╩═══════════════╩═══════════════╩═══════════════════╣\n";
        std::cout << "║                              SPEED (microseconds)                          ║\n";
        std::cout << "╠════════════════════════╦═══════════════════════╦═══════════════════════════╣\n";
        std::cout << "║         Method         ║   Mean Time (µs)      ║   Speed vs Essential      ║\n";
        std::cout << "╠════════════════════════╬═══════════════════════╬═══════════════════════════╣\n";
        
        double em_time = em_stats.mean_time > 0 ? em_stats.mean_time : 1;
        printSpeedRow("Essential Matrix", em_stats, em_time);
        printSpeedRow("Homography", hom_stats, em_time);
        printSpeedRow("Avg Displacement", avg_stats, em_time);
        printSpeedRow("Median Displacement", med_stats, em_time);
        printSpeedRow("RANSAC Displacement", ransac_stats, em_time);
        printSpeedRow("Hybrid EM+Avg", hybrid_stats, em_time);
        printSpeedRow("Direction Cue", dir_stats, em_time);
        
        std::cout << "╠════════════════════════╩═══════════════════════╩═══════════════════════════╣\n";
        std::cout << "║                                 RECOMMENDATION                             ║\n";
        std::cout << "╠════════════════════════════════════════════════════════════════════════════╣\n";
        
        // Find the best method by mean error
        struct MethodStats { std::string name; double mean_err; double mean_time; };
        std::vector<MethodStats> all_methods = {
            {"Essential Matrix", em_stats.mean_error, em_stats.mean_time},
            {"Homography", hom_stats.mean_error, hom_stats.mean_time},
            {"Avg Displacement", avg_stats.mean_error, avg_stats.mean_time},
            {"Median Displacement", med_stats.mean_error, med_stats.mean_time},
            {"RANSAC Displacement", ransac_stats.mean_error, ransac_stats.mean_time},
            {"Hybrid EM+Avg", hybrid_stats.mean_error, hybrid_stats.mean_time},
            {"Direction Cue", dir_stats.mean_error, dir_stats.mean_time}
        };
        
        auto best = std::min_element(all_methods.begin(), all_methods.end(),
            [](const MethodStats& a, const MethodStats& b) { return a.mean_err < b.mean_err; });
        
        std::string recommendation = best->name + ": Best accuracy (" + 
            std::to_string(best->mean_err).substr(0, 5) + " deg mean error)";
        
        std::cout << "║  " << std::setw(75) << std::left << recommendation << "║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        
        std::cout << "║  * Direction Cue uses its own GT (angle to target)          ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        
        // Print some sample comparisons
        std::cout << "Sample Comparisons (first 10 pairs):\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────\n";
        std::cout << std::setw(6) << "Pair" 
                  << std::setw(8) << "Match"
                  << std::setw(10) << "GT_rel"
                  << std::setw(10) << "GT_dir"
                  << std::setw(10) << "EM"
                  << std::setw(10) << "Hom"
                  << std::setw(10) << "Avg"
                  << std::setw(10) << "Median"
                  << std::setw(10) << "RANSAC"
                  << std::setw(10) << "Hybrid"
                  << std::setw(10) << "DirCue"
                  << std::setw(10) << "Med_err"
                  << std::setw(10) << "Dir_err" << "\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────\n";
        
        for (size_t i = 0; i < std::min(size_t(10), results_.size()); ++i) {
            const auto& r = results_[i];
            double gt = r.ground_truth_yaw * 180.0 / M_PI;
            double gt_dir = r.ground_truth_direction * 180.0 / M_PI;
            double em = r.essential_matrix.yaw_rad * 180.0 / M_PI;
            double hom = r.homography.yaw_rad * 180.0 / M_PI;
            double avg = r.avg_displacement.yaw_rad * 180.0 / M_PI;
            double med = r.median_displacement.yaw_rad * 180.0 / M_PI;
            double ran = r.ransac_displacement.yaw_rad * 180.0 / M_PI;
            double hyb = r.hybrid_em_avg.yaw_rad * 180.0 / M_PI;
            double dir = r.direction_cue.yaw_rad * 180.0 / M_PI;
            
            auto calcError = [](double val, double gt) {
                double err = std::abs(val - gt);
                if (err > 180) err = 360 - err;
                return err;
            };
            
            std::cout << std::setw(6) << r.frame_pair
                      << std::setw(8) << r.num_matches
                      << std::setw(10) << gt
                      << std::setw(10) << gt_dir
                      << std::setw(10) << em
                      << std::setw(10) << hom
                      << std::setw(10) << avg
                      << std::setw(10) << med
                      << std::setw(10) << ran
                      << std::setw(10) << hyb
                      << std::setw(10) << dir
                      << std::setw(10) << calcError(med, gt)
                      << std::setw(10) << calcError(dir, gt_dir) << "\n";
        }
        std::cout << "\n";
        
        // Debug info: Show displacement vectors
        std::cout << "Debug Info - Body Frame Displacements (first 5 pairs):\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────────\n";
        std::cout << std::setw(6) << "Pair" 
                  << std::setw(12) << "DroneYaw"
                  << std::setw(12) << "dX_global"
                  << std::setw(12) << "dY_global"
                  << std::setw(12) << "dX_body"
                  << std::setw(12) << "dY_body"
                  << std::setw(12) << "GT_yaw" << "\n";
        std::cout << "───────────────────────────────────────────────────────────────────────────────\n";
        
        for (size_t i = 0; i < std::min(size_t(5), results_.size()); ++i) {
            const auto& r = results_[i];
            std::cout << std::setw(6) << r.frame_pair
                      << std::setw(12) << (r.drone_yaw * 180.0 / M_PI)
                      << std::setw(12) << r.dx_global
                      << std::setw(12) << r.dy_global
                      << std::setw(12) << r.dx_body
                      << std::setw(12) << r.dy_body
                      << std::setw(12) << (r.ground_truth_yaw * 180.0 / M_PI) << "\n";
        }
        std::cout << "\n";
        
        // Analysis hint
        std::cout << "Analysis:\n";
        std::cout << "  - GT_rel: Ground truth for relative displacement methods (motion direction)\n";
        std::cout << "  - GT_dir: Ground truth for direction cue (angle to next waypoint)\n";
        std::cout << "  - Direction Cue measures WHERE features are in frame, not HOW they moved\n";
        std::cout << "  - If features are centered → target is ahead → angle ≈ 0°\n";
        std::cout << "  - If features are to the right → target is right → angle < 0°\n";
        std::cout << "  - If features are to the left → target is left → angle > 0°\n";
        std::cout << "\n";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YawMethodTester>();
    rclcpp::shutdown();
    return 0;
}
