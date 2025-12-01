#ifndef EXTRA_RVIZ_PANEL__DRONE_PANEL_HPP_
#define EXTRA_RVIZ_PANEL__DRONE_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>

namespace extra_rviz_panel
{

class DronePanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit DronePanel(QWidget * parent = nullptr);
    virtual ~DronePanel();

    void onInitialize() override;
    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;
    void arm();

protected Q_SLOTS:
    void updateParameters();
    void onTuningModeChanged(int state);
    void onRefreshClicked();
    void onSaveClicked();

protected:
    // UI Elements
    QCheckBox * tuning_mode_checkbox_;
    QPushButton * arm_button_;
    
    // Altitude
    QDoubleSpinBox * kp_alt_spin_;
    QDoubleSpinBox * ki_alt_spin_;
    QDoubleSpinBox * kd_alt_spin_;
    QDoubleSpinBox * setpoint_alt_spin_;

    // Roll
    QDoubleSpinBox * kp_roll_spin_;
    QDoubleSpinBox * ki_roll_spin_;
    QDoubleSpinBox * kd_roll_spin_;
    QDoubleSpinBox * setpoint_roll_spin_;

    // Pitch
    QDoubleSpinBox * kp_pitch_spin_;
    QDoubleSpinBox * ki_pitch_spin_;
    QDoubleSpinBox * kd_pitch_spin_;
    QDoubleSpinBox * setpoint_pitch_spin_;

    // Yaw
    QDoubleSpinBox * kp_yaw_spin_;
    QDoubleSpinBox * ki_yaw_spin_;
    QDoubleSpinBox * kd_yaw_spin_;
    QDoubleSpinBox * setpoint_yaw_spin_;

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;

    void createPidGroup(const QString & title, 
                       QDoubleSpinBox*& kp, QDoubleSpinBox*& ki, QDoubleSpinBox*& kd, QDoubleSpinBox*& setpoint,
                       QVBoxLayout * parent_layout);
    
    void loadPidValues();

    void setSpinBoxStyle(QDoubleSpinBox * spin);

private Q_SLOTS:
    void onArmClicked();
};

} // namespace extra_rviz_panel

#endif // EXTRA_RVIZ_PANEL__DRONE_PANEL_HPP_
