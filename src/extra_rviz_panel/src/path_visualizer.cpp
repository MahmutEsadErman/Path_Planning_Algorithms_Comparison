#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>

class RvizVisualizationsNode : public rclcpp::Node
{
public:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;

    // Path messages
    nav_msgs::msg::Path real_path_msg_;
    nav_msgs::msg::Path target_path_msg_;

    // Pose data
    std::shared_ptr<geometry_msgs::msg::Pose> drone_pose_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pose_;
    std::shared_ptr<geometry_msgs::msg::Point> starting_point_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Initializes the node, publishers, subscribers, and service clients.
     */
    RvizVisualizationsNode() : Node("rviz_visualization_node")
    {
        // QoS profile for MAVROS compatibility
        auto qos_profile = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info",
            qos_profile,
            std::bind(&RvizVisualizationsNode::state_callback, this, std::placeholders::_1)
        );

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose",
            10,
            std::bind(&RvizVisualizationsNode::target_pose_callback, this, std::placeholders::_1)
        );

        // Publishers
        real_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/drone/real_path",
            10
        );

        target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/drone/target_path",
            10
        );

        // Initialize path messages
        real_path_msg_.header.stamp = this->get_clock()->now();
        real_path_msg_.header.frame_id = "map";

        target_path_msg_.header.stamp = this->get_clock()->now();
        target_path_msg_.header.frame_id = "map";

        // Give publishers time to establish connections
        RCLCPP_INFO(this->get_logger(), "Waiting for publishers to be ready...");

        // Initialize drone_pose
        drone_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
        target_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

        // Timer to publish at 1Hz (required for attitude control)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RvizVisualizationsNode::send_path, this)
        );

        RCLCPP_INFO(this->get_logger(), "Rviz Visualizations Node initialized");
    }

private:
    /**
     * @brief Callback to receive pose data and republish it.
     */
    void state_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Assuming the desired pose is the third one in the array
        if (msg->poses.size() > 2) {
            drone_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->poses[2]);
            if (starting_point_ == nullptr) {
                starting_point_ = std::make_shared<geometry_msgs::msg::Point>(msg->poses[2].position);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "PoseArray does not contain enough poses.");
        }
    }

    /**
     * @brief Callback to receive target pose and add to target path.
     */
    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Wait until we have a starting point
        if (starting_point_ == nullptr) {
            return;
        }
        
        geometry_msgs::msg::PoseStamped stamped_pose;
        stamped_pose.header.frame_id = "map";
        stamped_pose.header.stamp = this->get_clock()->now();
        stamped_pose.pose = *msg;
        
        target_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(stamped_pose);

        // Offset the target pose by the starting point
        target_pose_->pose.position.x -= starting_point_->x;
        target_pose_->pose.position.y -= starting_point_->y;
        target_path_msg_.poses.push_back(*target_pose_);
        target_path_pub_->publish(target_path_msg_);
    }

    /**
     * @brief Sends the path data.
     */
    void send_path()
    {
        // Wait until we have received the first pose and established starting point
        if (starting_point_ == nullptr) {
            return;
        }

        geometry_msgs::msg::PoseStamped stamped_pose;
        stamped_pose.pose = *drone_pose_;
        
        // Offset the drone pose by the starting point using the local copy
        stamped_pose.pose.position.x -= starting_point_->x;
        stamped_pose.pose.position.y -= starting_point_->y;

        stamped_pose.header.frame_id = "map";
        stamped_pose.header.stamp = this->get_clock()->now();
        real_path_msg_.poses.push_back(stamped_pose);

        real_path_pub_->publish(real_path_msg_);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RvizVisualizationsNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
