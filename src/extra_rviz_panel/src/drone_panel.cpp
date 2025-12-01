#include "extra_rviz_panel/drone_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <QFile>
#include <QTextStream>
#include <QDir>

namespace extra_rviz_panel
{

DronePanel::DronePanel(QWidget * parent)
    : rviz_common::Panel(parent)
{
    QVBoxLayout * main_layout = new QVBoxLayout;

    // Tuning Mode Checkbox
    tuning_mode_checkbox_ = new QCheckBox("Enable Tuning Mode");
    main_layout->addWidget(tuning_mode_checkbox_);
    connect(tuning_mode_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(onTuningModeChanged(int)));

    // Arm Button
    arm_button_ = new QPushButton("ARM Drone");
    main_layout->addWidget(arm_button_);
    connect(arm_button_, SIGNAL(clicked()), this, SLOT(onArmClicked()));

    // PID Groups
    createPidGroup("Altitude", kp_alt_spin_, ki_alt_spin_, kd_alt_spin_, setpoint_alt_spin_, main_layout);
    createPidGroup("Roll", kp_roll_spin_, ki_roll_spin_, kd_roll_spin_, setpoint_roll_spin_, main_layout);
    createPidGroup("Pitch", kp_pitch_spin_, ki_pitch_spin_, kd_pitch_spin_, setpoint_pitch_spin_, main_layout);
    createPidGroup("Yaw", kp_yaw_spin_, ki_yaw_spin_, kd_yaw_spin_, setpoint_yaw_spin_, main_layout);

    setLayout(main_layout);
}

DronePanel::~DronePanel()
{
}

void DronePanel::onInitialize()
{
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/drone_controller_node");
    mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    
    // Try to load saved values
    loadPidValues();

    onTuningModeChanged(tuning_mode_checkbox_->checkState());
    updateParameters();
}

// ... (keep loadPidValues and createPidGroup as is, I will skip them in replacement if possible, but I need to replace the constructor and onInitialize)

// I will use a separate replace for arm() implementation at the end of file.


void DronePanel::loadPidValues()
{
    QString path = QDir::homePath() + "/.ros/drone_panel_pids.txt";
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(" ");
        if (parts.size() < 2) continue;
        
        QString key = parts[0];
        double value = parts[1].toDouble();
        
        if (key == "Kp_alt") kp_alt_spin_->setValue(value);
        else if (key == "Ki_alt") ki_alt_spin_->setValue(value);
        else if (key == "Kd_alt") kd_alt_spin_->setValue(value);
        else if (key == "setpoint_alt") setpoint_alt_spin_->setValue(value);
        
        else if (key == "Kp_roll") kp_roll_spin_->setValue(value);
        else if (key == "Ki_roll") ki_roll_spin_->setValue(value);
        else if (key == "Kd_roll") kd_roll_spin_->setValue(value);
        else if (key == "setpoint_roll") setpoint_roll_spin_->setValue(value);

        else if (key == "Kp_pitch") kp_pitch_spin_->setValue(value);
        else if (key == "Ki_pitch") ki_pitch_spin_->setValue(value);
        else if (key == "Kd_pitch") kd_pitch_spin_->setValue(value);
        else if (key == "setpoint_pitch") setpoint_pitch_spin_->setValue(value);

        else if (key == "Kp_yaw") kp_yaw_spin_->setValue(value);
        else if (key == "Ki_yaw") ki_yaw_spin_->setValue(value);
        else if (key == "Kd_yaw") kd_yaw_spin_->setValue(value);
        else if (key == "setpoint_yaw") setpoint_yaw_spin_->setValue(value);
    }
}

void DronePanel::createPidGroup(const QString & title, 
                               QDoubleSpinBox*& kp, QDoubleSpinBox*& ki, QDoubleSpinBox*& kd, QDoubleSpinBox*& setpoint,
                               QVBoxLayout * parent_layout)
{
    QGroupBox * group = new QGroupBox(title);
    QGridLayout * grid = new QGridLayout;

    kp = new QDoubleSpinBox; setSpinBoxStyle(kp);
    ki = new QDoubleSpinBox; setSpinBoxStyle(ki);
    kd = new QDoubleSpinBox; setSpinBoxStyle(kd);
    setpoint = new QDoubleSpinBox; setSpinBoxStyle(setpoint);

    grid->addWidget(new QLabel("Kp:"), 0, 0); grid->addWidget(kp, 0, 1);
    grid->addWidget(new QLabel("Ki:"), 0, 2); grid->addWidget(ki, 0, 3);
    grid->addWidget(new QLabel("Kd:"), 1, 0); grid->addWidget(kd, 1, 1);
    grid->addWidget(new QLabel("Set:"), 1, 2); grid->addWidget(setpoint, 1, 3);

    group->setLayout(grid);
    parent_layout->addWidget(group);

    // Connect signals
    connect(kp, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
    connect(ki, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
    connect(kd, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
    connect(setpoint, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
}

void DronePanel::setSpinBoxStyle(QDoubleSpinBox * spin)
{
    spin->setDecimals(4);
    spin->setSingleStep(0.01);
    spin->setRange(-1000.0, 1000.0); // Default range, override for setpoint if needed
}

void DronePanel::onTuningModeChanged(int state)
{
    if (!param_client_) return;
    
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("tuning_mode", state == Qt::Checked));
    param_client_->set_parameters(params);
}

void DronePanel::updateParameters()
{
    if (!param_client_) return;

    std::vector<rclcpp::Parameter> params;
    
    // Altitude
    params.push_back(rclcpp::Parameter("Kp_alt", kp_alt_spin_->value()));
    params.push_back(rclcpp::Parameter("Ki_alt", ki_alt_spin_->value()));
    params.push_back(rclcpp::Parameter("Kd_alt", kd_alt_spin_->value()));
    params.push_back(rclcpp::Parameter("setpoint_alt", setpoint_alt_spin_->value()));

    // Roll
    params.push_back(rclcpp::Parameter("setpoint_roll", setpoint_roll_spin_->value()));

    // Pitch
    params.push_back(rclcpp::Parameter("setpoint_pitch", setpoint_pitch_spin_->value()));

    // Yaw
    params.push_back(rclcpp::Parameter("Kp_yaw", kp_yaw_spin_->value()));
    params.push_back(rclcpp::Parameter("Ki_yaw", ki_yaw_spin_->value()));
    params.push_back(rclcpp::Parameter("Kd_yaw", kd_yaw_spin_->value()));
    params.push_back(rclcpp::Parameter("setpoint_yaw", setpoint_yaw_spin_->value()));

    param_client_->set_parameters(params);
}

void DronePanel::load(const rviz_common::Config & config)
{
    rviz_common::Panel::load(config);

    bool tuning_mode;
    if(config.mapGetBool("TuningMode", &tuning_mode)) {
        tuning_mode_checkbox_->setChecked(tuning_mode);
    }

    float val;
    // Altitude
    if(config.mapGetFloat("Kp_alt", &val)) kp_alt_spin_->setValue(val);
    if(config.mapGetFloat("Ki_alt", &val)) ki_alt_spin_->setValue(val);
    if(config.mapGetFloat("Kd_alt", &val)) kd_alt_spin_->setValue(val);
    if(config.mapGetFloat("setpoint_alt", &val)) setpoint_alt_spin_->setValue(val);

    // Roll
    if(config.mapGetFloat("Kp_roll", &val)) kp_roll_spin_->setValue(val);
    if(config.mapGetFloat("Ki_roll", &val)) ki_roll_spin_->setValue(val);
    if(config.mapGetFloat("Kd_roll", &val)) kd_roll_spin_->setValue(val);
    if(config.mapGetFloat("setpoint_roll", &val)) setpoint_roll_spin_->setValue(val);

    // Pitch
    if(config.mapGetFloat("Kp_pitch", &val)) kp_pitch_spin_->setValue(val);
    if(config.mapGetFloat("Ki_pitch", &val)) ki_pitch_spin_->setValue(val);
    if(config.mapGetFloat("Kd_pitch", &val)) kd_pitch_spin_->setValue(val);
    if(config.mapGetFloat("setpoint_pitch", &val)) setpoint_pitch_spin_->setValue(val);

    // Yaw
    if(config.mapGetFloat("Kp_yaw", &val)) kp_yaw_spin_->setValue(val);
    if(config.mapGetFloat("Ki_yaw", &val)) ki_yaw_spin_->setValue(val);
    if(config.mapGetFloat("Kd_yaw", &val)) kd_yaw_spin_->setValue(val);
    if(config.mapGetFloat("setpoint_yaw", &val)) setpoint_yaw_spin_->setValue(val);
}

void DronePanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);

    config.mapSetValue("TuningMode", tuning_mode_checkbox_->isChecked());

    // Altitude
    config.mapSetValue("Kp_alt", kp_alt_spin_->value());
    config.mapSetValue("Ki_alt", ki_alt_spin_->value());
    config.mapSetValue("Kd_alt", kd_alt_spin_->value());
    config.mapSetValue("setpoint_alt", setpoint_alt_spin_->value());

    // Roll
    config.mapSetValue("Kp_roll", kp_roll_spin_->value());
    config.mapSetValue("Ki_roll", ki_roll_spin_->value());
    config.mapSetValue("Kd_roll", kd_roll_spin_->value());
    config.mapSetValue("setpoint_roll", setpoint_roll_spin_->value());

    // Pitch
    config.mapSetValue("Kp_pitch", kp_pitch_spin_->value());
    config.mapSetValue("Ki_pitch", ki_pitch_spin_->value());
    config.mapSetValue("Kd_pitch", kd_pitch_spin_->value());
    config.mapSetValue("setpoint_pitch", setpoint_pitch_spin_->value());

    // Yaw
    config.mapSetValue("Kp_yaw", kp_yaw_spin_->value());
    config.mapSetValue("Ki_yaw", ki_yaw_spin_->value());
    config.mapSetValue("Kd_yaw", kd_yaw_spin_->value());
    config.mapSetValue("setpoint_yaw", setpoint_yaw_spin_->value());
}

void DronePanel::onArmClicked()
{   
    if (!mode_client_) return;

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "ALT_HOLD";

    if (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "SetMode service not available");
        return;
    }

    mode_client_->async_send_request(request);

    arm();
}

void DronePanel::arm()
{
    if (!param_client_ || !arming_client_) return;
    
    // Call arming service
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    
    if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Arming service not available");
        return;
    }

    auto future = arming_client_->async_send_request(request);
    // We don't block here to avoid freezing UI

    // Set hover throttle first
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("hover_throttle", 500.0));
    param_client_->set_parameters(params);
}

} // namespace extra_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(extra_rviz_panel::DronePanel, rviz_common::Panel)
