/**
 * @file feature_processor.hpp
 * @brief Feature detection, matching, and motion estimation utilities
 */

#ifndef GPS_DENIED_NAV_FEATURE_PROCESSOR_HPP
#define GPS_DENIED_NAV_FEATURE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <string>
#include <vector>

namespace gps_denied_nav {

/**
 * @brief Class for handling visual feature detection, matching, and motion estimation
 */
class FeatureProcessor {
public:
    /**
     * @brief Constructor
     * @param feature_detector Type of feature detector: "ORB", "SURF", or "SIFT"
     * @param camera_matrix Camera intrinsic matrix K
     * @param camera_pitch_angle Camera pitch angle in degrees
     */
    FeatureProcessor(const std::string& feature_detector, 
                     const cv::Mat& camera_matrix,
                     const double camera_pitch_angle);

    /**
     * @brief Detect and compute features from a grayscale image
     * @param gray_image Input grayscale image
     * @param keypoints Output keypoints
     * @param descriptors Output descriptors
     */
    void detectAndCompute(const cv::Mat& gray_image,
                          std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors);

    /**
     * @brief Compare features between two descriptor sets
     * @param des1 First descriptor set
     * @param des2 Second descriptor set
     * @param good_matches Output good matches after ratio test
     * @param ratio_threshold Ratio test threshold (default 0.75)
     */
    void compareFeatures(const cv::Mat& des1, 
                         const cv::Mat& des2, 
                         std::vector<cv::DMatch>& good_matches,
                         float ratio_threshold = 0.75f);

    /**
     * @brief Calculate relative yaw angle from feature matches
     * @param kp1 Keypoints from first image
     * @param kp2 Keypoints from second image
     * @param good_matches Matched features
     * @return Target yaw angle in radians
     */
    double calculateRelativeYaw(const std::vector<cv::KeyPoint>& kp1,
                                const std::vector<cv::KeyPoint>& kp2,
                                const std::vector<cv::DMatch>& good_matches);

private:
    cv::Ptr<cv::Feature2D> fe_method_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    cv::Mat K_;
    cv::Mat cam_tf_;
    double camera_pitch_angle_;
};

}  // namespace gps_denied_nav

#endif  // GPS_DENIED_NAV_FEATURE_PROCESSOR_HPP
