/**
 * @file main.cpp
 * @brief Entry point for GPS-denied navigation node
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include "gps_denied_nav/follow_path_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gps_denied_nav::FollowPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}