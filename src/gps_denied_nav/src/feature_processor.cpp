/**
 * @file feature_processor.cpp
 * @brief Implementation of feature detection, matching, and motion estimation
 */

#include "gps_denied_nav/feature_processor.hpp"
#include <cmath>

namespace gps_denied_nav {

FeatureProcessor::FeatureProcessor(const std::string& feature_detector,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& camera_transform)
    : K_(camera_matrix.clone()), cam_tf_(camera_transform.clone())
{
    // Initialize feature detector and matcher based on detector type
    if (feature_detector == "ORB") {
        fe_method_ = cv::ORB::create();
        // Use LSH Index for binary descriptors (ORB)
        matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    } else if (feature_detector == "SURF") {
        int hessian_threshold = 400;
        fe_method_ = cv::xfeatures2d::SURF::create(hessian_threshold);
        matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::KDTreeIndexParams>(5));
    } else {
        // Default to SIFT
        fe_method_ = cv::SIFT::create();
        matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::KDTreeIndexParams>(5));
    }
}

void FeatureProcessor::detectAndCompute(const cv::Mat& gray_image,
                                         std::vector<cv::KeyPoint>& keypoints,
                                         cv::Mat& descriptors)
{
    fe_method_->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);
}

void FeatureProcessor::compareFeatures(const cv::Mat& des1,
                                        const cv::Mat& des2,
                                        std::vector<cv::DMatch>& good_matches,
                                        float ratio_threshold)
{
    if (des1.rows < 2 || des2.rows < 2) {
        return;
    }

    // Feature Matching (FLANN) with k=2 for ratio test
    std::vector<std::vector<cv::DMatch>> matches;
    matcher_->knnMatch(des1, des2, matches, 2);

    // Ratio Test
    for (const auto& match_pair : matches) {
        if (match_pair.size() == 2 && 
            match_pair[0].distance < ratio_threshold * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }
}

double FeatureProcessor::calculateRelativeYaw(const std::vector<cv::KeyPoint>& kp1,
                                               const std::vector<cv::KeyPoint>& kp2,
                                               const std::vector<cv::DMatch>& good_matches)
{
    if (good_matches.empty()) {
        return 0.0;
    }

    // Extract matched points
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(good_matches.size());
    pts2.reserve(good_matches.size());
    
    for (const auto& m : good_matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }
    
    // Normalize points using undistortPoints
    // This converts pixel coords to normalized camera coords using K^(-1)
    std::vector<cv::Point2f> pts1_norm, pts2_norm;
    cv::undistortPoints(pts1, pts1_norm, K_, cv::noArray());
    cv::undistortPoints(pts2, pts2_norm, K_, cv::noArray());
    
    // Compute per-match displacement vectors
    std::vector<double> dx_vals, dy_vals;
    dx_vals.reserve(pts1_norm.size());
    dy_vals.reserve(pts1_norm.size());
    
    for (size_t i = 0; i < pts1_norm.size(); ++i) {
        dx_vals.push_back(pts2_norm[i].x - pts1_norm[i].x);
        dy_vals.push_back(pts2_norm[i].y - pts1_norm[i].y);
    }
    
    // Use median instead of mean for outlier robustness
    std::sort(dx_vals.begin(), dx_vals.end());
    std::sort(dy_vals.begin(), dy_vals.end());
    
    double median_dx = dx_vals[dx_vals.size() / 2];
    double median_dy = dy_vals[dy_vals.size() / 2];
    
    // Create displacement vector in camera frame
    cv::Mat t_cam = (cv::Mat_<double>(3, 1) << -median_dx, -median_dy, 1.0);
    
    // Transform to body frame: t_body = cam_tf_ * t_cam
    cv::Mat t_body = cam_tf_ * t_cam;
    
    // Calculate target angle from translation vector
    double target_angle = std::atan2(t_body.at<double>(1), t_body.at<double>(0));

    return target_angle;
}

double FeatureProcessor::calculateRelativeRotation(const std::vector<cv::KeyPoint>& kp1,
                                                    const std::vector<cv::KeyPoint>& kp2,
                                                    const std::vector<cv::DMatch>& good_matches)
{
    if (good_matches.size() < 4) {
        return 0.0;
    }

    // Extract matched points
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(good_matches.size());
    pts2.reserve(good_matches.size());
    
    for (const auto& m : good_matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    // Estimate similarity transform (rotation + scale + translation)
    // This is robust to outliers via RANSAC internally
    cv::Mat inliers;
    cv::Mat transform = cv::estimateAffinePartial2D(pts1, pts2, inliers,
                                                      cv::RANSAC, 3.0);
    
    if (transform.empty()) {
        return 0.0;
    }

    // The similarity transform matrix is:
    // [s*cos(θ)  -s*sin(θ)  tx]
    // [s*sin(θ)   s*cos(θ)  ty]
    // Extract rotation angle θ
    double a = transform.at<double>(0, 0);  // s*cos(θ)
    double b = transform.at<double>(1, 0);  // s*sin(θ)
    double rotation_angle = std::atan2(b, a);

    // For a downward-facing camera:
    // Image rotation is opposite to drone yaw rotation
    // (when drone turns CW, features appear to rotate CCW in image)
    return -rotation_angle;
}

}  // namespace gps_denied_nav
