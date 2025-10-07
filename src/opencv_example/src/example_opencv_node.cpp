#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.hpp> // Updated: Jazzy provides cv_bridge.hpp (not cv_bridge.h) for ROS <-> OpenCV conversions.
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer( 500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3); // 8-bit unsigned, 3 channels

    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image).toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_);
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

