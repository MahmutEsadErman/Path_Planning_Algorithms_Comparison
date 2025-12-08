#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include <cmath>

class VelYawCommander : public rclcpp::Node
{
public:
    VelYawCommander() : Node("vel_yaw_commander")
    {
        pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);



        // Build message template
        msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        msg_.type_mask =
              mavros_msgs::msg::PositionTarget::IGNORE_PX
            | mavros_msgs::msg::PositionTarget::IGNORE_PY
            | mavros_msgs::msg::PositionTarget::IGNORE_PZ
            | mavros_msgs::msg::PositionTarget::IGNORE_AFX
            | mavros_msgs::msg::PositionTarget::IGNORE_AFY
            | mavros_msgs::msg::PositionTarget::IGNORE_AFZ
            | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        // Velocity 5 m/s forward
        msg_.velocity.x = 5.0;
        msg_.velocity.y = 0.0;
        msg_.velocity.z = 0.0;

        // Yaw = 20 degrees (convert to radians)
        msg_.yaw = 20.0 * M_PI / 180.0;
    }

private:
    void timerCallback()
    {
        pub_->publish(msg_);
    }

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    mavros_msgs::msg::PositionTarget msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelYawCommander>());
    rclcpp::shutdown();
    return 0;
}
