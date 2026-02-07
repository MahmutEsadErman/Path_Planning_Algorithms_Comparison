/**
 * @file types.hpp
 * @brief Common data structures for GPS-denied navigation
 */

#ifndef GPS_DENIED_NAV_TYPES_HPP
#define GPS_DENIED_NAV_TYPES_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace gps_denied_nav {

/**
 * @brief Structure to hold visual features (keypoints and descriptors)
 */
struct Features {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

/**
 * @brief Structure to hold complete frame data including features and sensor data
 */
struct FrameData {
    Features features;
    sensor_msgs::msg::Imu imu;
    std_msgs::msg::Float64 altitude;
    geometry_msgs::msg::Pose target_pose;
};

}  // namespace gps_denied_nav

#endif  // GPS_DENIED_NAV_TYPES_HPP
