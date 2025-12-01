/**
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <memory>

#include "gps_denied_nav/drone_mav_connection.hpp"

class DroneControllerNode : public rclcpp::Node {
public:
    DroneControllerNode() 
        : Node("drone_controller_node")
    {
        // MAVLink bağlantısını başlat
        try {
            mavlink_conn_ = std::make_unique<MAVLinkConnection>("127.0.0.1", 14551);
            
            RCLCPP_INFO(this->get_logger(), "Waiting for heartbeat...");
            if (!mavlink_conn_->wait_heartbeat()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive heartbeat!");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Connected to drone (System ID: %d)", 
                       mavlink_conn_->get_target_system());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Connection error: %s", e.what());
            return;
        }

        // QoS profile for MAVROS topics (Best Effort + larger queue)
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Subscribers
        cmd_move_sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
            "drone/cmd_move", 10,
            std::bind(&DroneControllerNode::cmdMoveCallback, this, std::placeholders::_1));
        

        alt_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", sensor_qos,
            std::bind(&DroneControllerNode::altitudeCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            std::bind(&DroneControllerNode::imuCallback, this, std::placeholders::_1));

        err_alt_pub_ = create_publisher<std_msgs::msg::Float64>("/drone/err_alt", 10);
        err_roll_pub_ = create_publisher<std_msgs::msg::Float64>("/drone/err_roll", 10);
        err_pitch_pub_ = create_publisher<std_msgs::msg::Float64>("/drone/err_pitch", 10);
        err_yaw_pub_ = create_publisher<std_msgs::msg::Float64>("/drone/err_yaw", 10);

        // PID params
        this->declare_parameter("Kp_alt", 3.0);
        this->declare_parameter("Ki_alt", 0.22);
        this->declare_parameter("Kd_alt", 0.1);
        
        // Roll PID
        this->declare_parameter("Kp_roll", 0.135);
        this->declare_parameter("Ki_roll", 0.135);
        this->declare_parameter("Kd_roll", 0.0036);
        
        // Pitch PID
        this->declare_parameter("Kp_pitch", 0.135);
        this->declare_parameter("Ki_pitch", 0.135);
        this->declare_parameter("Kd_pitch", 0.0036);
        
        // Yaw PID
        this->declare_parameter("Kp_yaw", 0.3);
        this->declare_parameter("Ki_yaw", 0.05);
        this->declare_parameter("Kd_yaw", 0.0);

        // Tuning Mode Parameters
        this->declare_parameter("tuning_mode", false);
        this->declare_parameter("hover_throttle", 500.0);
        this->declare_parameter("setpoint_alt", 0.0);
        this->declare_parameter("setpoint_roll", 0.0);
        this->declare_parameter("setpoint_pitch", 0.0);
        this->declare_parameter("setpoint_yaw", 0.0);

        getParams();

        // Register parameter callback
        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DroneControllerNode::parametersCallback, this, std::placeholders::_1));

        // Initialize last_time_ with the correct clock source
        last_time_ = this->now();

        // Timer for sending manual control (20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DroneControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Drone Controller Node started!");
        RCLCPP_INFO(this->get_logger(), "Subscribe to: drone/cmd_move (mavros_msgs/ManualControl, instead of throttle use altitude)");
    }

private:
    void cmdMoveCallback(const mavros_msgs::msg::ManualControl::SharedPtr msg) {
        if (!tuning_mode_) {
            target_pitch_ = msg->x;
            target_roll_ = msg->y;
            target_altitude_ = msg->z;
            target_yaw_ = msg->r;
        }
    }

    void sendManualControl(float pitch, float roll, float throttle, float yaw) {
        if (mavlink_conn_) {
            mavlink_conn_->send_manual_control(
                pitch,
                roll,
                throttle,
                yaw
            );
        }
    }

    void getParams()
    {
        Kp_alt_ = get_parameter("Kp_alt").as_double();
        Ki_alt_ = get_parameter("Ki_alt").as_double();
        Kd_alt_ = get_parameter("Kd_alt").as_double();
        
        Kp_roll_ = get_parameter("Kp_roll").as_double();
        Ki_roll_ = get_parameter("Ki_roll").as_double();
        Kd_roll_ = get_parameter("Kd_roll").as_double();
        
        Kp_pitch_ = get_parameter("Kp_pitch").as_double();
        Ki_pitch_ = get_parameter("Ki_pitch").as_double();
        Kd_pitch_ = get_parameter("Kd_pitch").as_double();
        
        Kp_yaw_ = get_parameter("Kp_yaw").as_double();
        Ki_yaw_ = get_parameter("Ki_yaw").as_double();
        Kd_yaw_ = get_parameter("Kd_yaw").as_double();

        tuning_mode_ = get_parameter("tuning_mode").as_bool();
        hover_throttle_ = get_parameter("hover_throttle").as_double();
        setpoint_alt_ = get_parameter("setpoint_alt").as_double();
        setpoint_roll_ = get_parameter("setpoint_roll").as_double();
        setpoint_pitch_ = get_parameter("setpoint_pitch").as_double();
        setpoint_yaw_ = get_parameter("setpoint_yaw").as_double();
    }

    // --- Callbacks ---
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "Kp_alt") Kp_alt_ = param.as_double();
            else if (param.get_name() == "Ki_alt") Ki_alt_ = param.as_double();
            else if (param.get_name() == "Kd_alt") Kd_alt_ = param.as_double();
            else if (param.get_name() == "Kp_roll") Kp_roll_ = param.as_double();
            else if (param.get_name() == "Ki_roll") Ki_roll_ = param.as_double();
            else if (param.get_name() == "Kd_roll") Kd_roll_ = param.as_double();
            else if (param.get_name() == "Kp_pitch") Kp_pitch_ = param.as_double();
            else if (param.get_name() == "Ki_pitch") Ki_pitch_ = param.as_double();
            else if (param.get_name() == "Kd_pitch") Kd_pitch_ = param.as_double();
            else if (param.get_name() == "Kp_yaw") Kp_yaw_ = param.as_double();
            else if (param.get_name() == "Ki_yaw") Ki_yaw_ = param.as_double();
            else if (param.get_name() == "Kd_yaw") Kd_yaw_ = param.as_double();
            else if (param.get_name() == "tuning_mode") tuning_mode_ = param.as_bool();
            else if (param.get_name() == "hover_throttle") hover_throttle_ = param.as_double();
            else if (param.get_name() == "setpoint_alt") setpoint_alt_ = param.as_double();
            else if (param.get_name() == "setpoint_roll") setpoint_roll_ = param.as_double();
            else if (param.get_name() == "setpoint_pitch") setpoint_pitch_ = param.as_double();
            else if (param.get_name() == "setpoint_yaw") setpoint_yaw_ = param.as_double();
        }

        return result;
    }

    void altitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_altitude_ = msg->data;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert quaternion to roll/pitch/yaw
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        current_roll_ = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            current_pitch_ = std::copysign(M_PI / 2, sinp);
        else
            current_pitch_ = std::asin(sinp);

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        // Store angular velocities for D-term
        current_ang_vel_x_ = msg->angular_velocity.x;
        current_ang_vel_y_ = msg->angular_velocity.y;
        current_ang_vel_z_ = msg->angular_velocity.z;
    }

    // --- Control Loop ---
    void controlLoop()
    {   
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Sanity check for dt
        if (dt <= 0.0 || dt > 0.5) {
            dt = 0.05; 
        }

        // Tuning Mode Logic
        if (tuning_mode_) {
            target_altitude_ = setpoint_alt_;
            target_roll_ = setpoint_roll_;
            target_pitch_ = setpoint_pitch_;
            target_yaw_ = setpoint_yaw_;
        }

        // Altitude PID
        double err_alt = target_altitude_ - current_altitude_;
        integral_alt_ += err_alt * dt;
        // anti wind-up
        integral_alt_ = clamp(integral_alt_, -10.0, 10.0);
        
        double deriv_alt = 0.7 * deriv_alt_prev + 0.3 * ((err_alt - prev_err_alt_) / dt);
        deriv_alt_prev = deriv_alt;
        prev_err_alt_ = err_alt;
        
        double throttle = Kp_alt_ * err_alt + Ki_alt_ * integral_alt_ + Kd_alt_ * deriv_alt;

        // Roll PID
        double err_roll = target_roll_ - current_roll_;
        integral_roll_ += err_roll * dt;
        integral_roll_ = clamp(integral_roll_, -5.0, 5.0); // Anti-windup
        
        // Derivative on Measurement: -Kd * gyro_rate
        // Note: gyro x is roll rate in body frame (approximate for small angles)
        double roll_out = Kp_roll_ * err_roll + Ki_roll_ * integral_roll_ - Kd_roll_ * current_ang_vel_x_;

        // Pitch PID
        double err_pitch = target_pitch_ - current_pitch_;
        integral_pitch_ += err_pitch * dt;
        integral_pitch_ = clamp(integral_pitch_, -5.0, 5.0); // Anti-windup
        
        // Derivative on Measurement
        double pitch_out = Kp_pitch_ * err_pitch + Ki_pitch_ * integral_pitch_ - Kd_pitch_ * current_ang_vel_y_;

        // Yaw PID
        double err_yaw = atan2(sin(target_yaw_ - current_yaw_),
                       cos(target_yaw_ - current_yaw_));
        integral_yaw_ += err_yaw * dt;
        integral_yaw_ = clamp(integral_yaw_, -5.0, 5.0); // Anti-windup
        
        // Derivative on Measurement
        double yaw_out = Kp_yaw_ * err_yaw + Ki_yaw_ * integral_yaw_ - Kd_yaw_ * current_ang_vel_z_;

        // Normalize outputs to [-900,900]
        auto clamp_val = [](double v, double min, double max) { return std::max(min, std::min(max, v)); };
        float x = clamp_val(pitch_out * 500, -900.0f, 900.0f);   // pitch
        float y = clamp_val(roll_out * 500, -900.0f, 900.0f);    // roll
        // Add feed-forward gravity compensation (approx 500)
        float z = clamp_val(throttle * 100 + hover_throttle_, 0.0f, 900.0f); // throttle
        // Invert Yaw for MAVLink (CW positive) vs ROS (CCW positive)
        float r = clamp_val(-yaw_out * 200, -900.0f, 900.0f);    // yaw

        sendManualControl(x, y, z, r);
        
        if (tuning_mode_) {
            plotErrors(err_alt, err_roll, err_pitch, err_yaw);
        }
        
        RCLCPP_INFO(get_logger(),
                    "Alt %.2f/%.2f | RPY [%.2f %.2f %.2f] | Thr %.1f | Mode: %s",
                    current_altitude_, target_altitude_,
                    current_roll_, current_pitch_, current_yaw_, z,
                    tuning_mode_ ? "TUNING" : "MANUAL");
    }

    double clamp(double v, double min, double max) { 
        return std::max(min, std::min(max, v));
    }
    
    void plotErrors(double err_alt, double err_roll, double err_pitch, double err_yaw) {
        std_msgs::msg::Float64 msg;
        
        msg.data = err_alt;
        err_alt_pub_->publish(msg);
        
        msg.data = err_roll;
        err_roll_pub_->publish(msg);
        
        msg.data = err_pitch;
        err_pitch_pub_->publish(msg);
        
        msg.data = err_yaw;
        err_yaw_pub_->publish(msg);
    }
    
    // --- ROS2 Components ---
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr cmd_move_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_alt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_yaw_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // --- MAVLink connection ---
    std::unique_ptr<MAVLinkConnection> mavlink_conn_;

    // --- Current States ---
    double current_altitude_ = 0.0;
    double current_roll_ = 0.0;
    double current_pitch_ = 0.0;
    double current_yaw_ = 0.0;
    double hover_throttle_ = 0.0;
    
    // Gyro rates for D-term
    double current_ang_vel_x_ = 0.0;
    double current_ang_vel_y_ = 0.0;
    double current_ang_vel_z_ = 0.0;

    // --- Target States ---
    double target_altitude_ = 0.0;
    double target_roll_ = 0.0;
    double target_pitch_ = 0.0;
    double target_yaw_ = 0.0;

    // --- PID Gains ---
    double Kp_alt_, Ki_alt_, Kd_alt_;
    double Kp_roll_, Ki_roll_, Kd_roll_;
    double Kp_pitch_, Ki_pitch_, Kd_pitch_;
    double Kp_yaw_, Ki_yaw_, Kd_yaw_;

    // --- Tuning Mode Params ---
    bool tuning_mode_ = false;
    double setpoint_alt_ = 0.0;
    double setpoint_roll_ = 0.0;
    double setpoint_pitch_ = 0.0;
    double setpoint_yaw_ = 0.0;

    // --- PID State ---
    rclcpp::Time last_time_;
    double prev_err_alt_ = 0.0, integral_alt_ = 0.0, deriv_alt_prev = 0.0;
    double integral_roll_ = 0.0;
    double integral_pitch_ = 0.0;
    double integral_yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}