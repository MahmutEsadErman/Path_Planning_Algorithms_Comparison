#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

class CameraVideoWriter : public rclcpp::Node
{
public:
  CameraVideoWriter()
  : Node("camera_video_writer"), frame_count_(0), is_initialized_(false)
  {
    // Subscribe to camera image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image",
      10,
      std::bind(&CameraVideoWriter::imageCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to /camera/image topic");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS2 image message to OpenCV Mat
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      
      // Initialize video writer on first frame
      if (!is_initialized_) {
        int width = frame.cols;
        int height = frame.rows;
        int fps = 30;
        
        // Define codec and create VideoWriter object
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        video_writer_.open("output_video.mp4", fourcc, fps, cv::Size(width, height));
        
        if (!video_writer_.isOpened()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open video writer");
          return;
        }
        
        is_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), 
          "Video writer initialized: %dx%d at %d fps", width, height, fps);
      }
      
      // Write frame to video
      if (video_writer_.isOpened()) {
        video_writer_.write(frame);
        frame_count_++;
        
        if (frame_count_ % 30 == 0) {
          RCLCPP_INFO(this->get_logger(), "Frames written: %ld", frame_count_);
        }
      }
    } 
    catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::VideoWriter video_writer_;
  long frame_count_;
  bool is_initialized_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraVideoWriter>());
  rclcpp::shutdown();
  return 0;
}