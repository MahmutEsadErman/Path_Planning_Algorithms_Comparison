#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

class FeatureDetectorNode : public rclcpp::Node {
public:
    FeatureDetectorNode() : Node("feature_detector_from_file") {
        // Determine default image path using ament_index_cpp so it works regardless of current working directory.
        std::string default_image_path;
        try {
            auto share_dir = ament_index_cpp::get_package_share_directory("opencv_example");
            default_image_path = share_dir + "/images/penguins.jpg";
        } catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "Could not resolve package share directory: %s. Falling back to relative path.", e.what());
            default_image_path = "images/penguins.jpg"; // fallback
        }
        // Declare parameters (user can override via CLI)
        this->declare_parameter<std::string>("image_path", default_image_path);

        featured_image_pub_ = image_transport::create_publisher(this, "orb_example/featured_image");

        orb_ = cv::ORB::create();
        
        while (rclcpp::ok()){
            auto out_msg = find_features();
            if (out_msg) {
                featured_image_pub_.publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "Published featured image.");
            }
        }
    }

private:
    sensor_msgs::msg::Image::SharedPtr find_features() {
        std::string image_path = this->get_parameter("image_path").as_string();
        if (image_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'image_path' is empty. Set it via --ros-args -p image_path:=/path/to/image.jpg");
            return nullptr;
        }

        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read image from path: %s", image_path.c_str());
            return nullptr;
        }

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

        cv::Mat output;
        cv::drawKeypoints(img, keypoints, output, cv::Scalar(0, 255, 0));

        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output).toImageMsg();
        if (out_msg) {
            return out_msg;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert output image to ROS message.");
            return nullptr;
        }
    }

    image_transport::Publisher featured_image_pub_;
    cv::Ptr<cv::ORB> orb_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureDetectorNode>();
    // No need to spin indefinitely; work done in constructor. Spin some to allow intra-process delivery if needed.
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin_some();
    rclcpp::shutdown();
    return 0;
}
