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
    // Get corresponding points
    std::vector<cv::Point2f> q1, q2;
    for (const auto& m : good_matches) {
        q1.push_back(kp1[m.queryIdx].pt);
        q2.push_back(kp2[m.trainIdx].pt);
    }

    // Estimate motion using Essential Matrix
    cv::Mat E, R, t, mask;
    
    // Use USAC_MAGSAC (modern RANSAC) with threshold 1.0 px
    E = cv::findEssentialMat(q1, q2, K_, cv::USAC_MAGSAC, 0.999, 1.0, mask);
    
    // Recover pose using only good inliers
    cv::recoverPose(E, q1, q2, K_, R, t, mask);
    
    // Transform translation: t_body = C * t_cam
    cv::Mat t_ros = cam_tf_ * t;

    // Calculate target angle from translation vector
    // atan2(y, x) where y=Y_component (left), x=X_component (forward)
    double target_angle = std::atan2(t_ros.at<double>(1), t_ros.at<double>(0));

    return target_angle;
}

}  // namespace gps_denied_nav
