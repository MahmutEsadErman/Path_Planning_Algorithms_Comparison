#include "extra_rviz_panel/rosbag2_panel.hpp"

#include <chrono>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QTimer>
#include <QSpinBox>
#include <QLabel>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rviz_common/display_context.hpp"

#include "vo_calculation.cpp"

using namespace std::chrono_literals;

namespace rviz_playback_panel
{

PlaybackPanel::PlaybackPanel(QWidget* parent)
  : rviz_common::Panel(parent),
    playback_timer_(nullptr),
    reader_(nullptr)
{
  // Get the node from RViz - will be initialized when onInitialize is called
  rviz_node_ = nullptr;

  // --- Create GUI Layout ---
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  
  // File selection layout
  QHBoxLayout* file_layout = new QHBoxLayout();
  file_path_editor_ = new QLineEdit("Click Browse to select ROS 2 bag directory...");
  file_path_editor_->setReadOnly(true);
  browse_button_ = new QPushButton("Browse");
  file_layout->addWidget(file_path_editor_);
  file_layout->addWidget(browse_button_);
  main_layout->addLayout(file_layout);

  // Control button layout
  QHBoxLayout* controls_layout = new QHBoxLayout();
  play_pause_button_ = new QPushButton("Play");
  stop_button_ = new QPushButton("Stop");
  play_pause_button_->setEnabled(false); // Disabled until bag is loaded
  stop_button_->setEnabled(false);
  controls_layout->addWidget(play_pause_button_);
  controls_layout->addWidget(stop_button_);
  main_layout->addLayout(controls_layout);

  // Timer interval control layout
  QHBoxLayout* timer_layout = new QHBoxLayout();
  timer_interval_label_ = new QLabel("Timer Interval (ms):");
  timer_interval_spinbox_ = new QSpinBox();
  choose_moment_button_ = new QPushButton("Choose Moment");
  timer_interval_spinbox_->setMinimum(1);
  timer_interval_spinbox_->setMaximum(1000);
  timer_interval_spinbox_->setValue(10);
  timer_interval_spinbox_->setToolTip("Set the QTimer interval in milliseconds");
  timer_layout->addWidget(timer_interval_label_);
  timer_layout->addWidget(timer_interval_spinbox_);
  timer_layout->addWidget(choose_moment_button_);
  timer_layout->addStretch(); // Add stretch to push widgets to the left
  main_layout->addLayout(timer_layout);

  // Set the main layout for the panel
  setLayout(main_layout);

  // --- Create Playback Timer ---
  // Use a QTimer for GUI-safe callbacks
  playback_timer_ = new QTimer(this);

  // --- Connect Signals & Slots ---
  connect(browse_button_, &QPushButton::clicked, this, &PlaybackPanel::onBrowseClicked);
  connect(play_pause_button_, &QPushButton::clicked, this, &PlaybackPanel::onPlayPauseClicked);
  connect(stop_button_, &QPushButton::clicked, this, &PlaybackPanel::onStopClicked);
  connect(choose_moment_button_, &QPushButton::clicked, this, &PlaybackPanel::onChooseMomentClicked);
  connect(playback_timer_, &QTimer::timeout, this, &PlaybackPanel::onTimerCallback);
  connect(timer_interval_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged), 
          this, &PlaybackPanel::onTimerIntervalChanged);

}

PlaybackPanel::~PlaybackPanel()
{
  // reader_ is a unique_ptr, it will be cleaned up automatically
}

void PlaybackPanel::onInitialize()
{
  // Get the ROS node from RViz's display context
  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (rviz_ros_node) {
    rviz_node_ = rviz_ros_node->get_raw_node();
  }
}

// Helper function to deserialize a message from SerializedBagMessage
template<typename MessageT>
std::shared_ptr<MessageT> deserializeMessage(const rosbag2_storage::SerializedBagMessageSharedPtr& serialized_bag_msg)
{
  if (!serialized_bag_msg) {
    return nullptr;
  }
  auto msg = std::make_shared<MessageT>();
  rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage serialized_msg(*(serialized_bag_msg->serialized_data));
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

void PlaybackPanel::onChooseMomentClicked()
{ 
  static int8_t counter = 0;
  static std::map<std::string, rosbag2_storage::SerializedBagMessageSharedPtr> prev_msg;
  static VisualOdometry vo_calculator;

  if (counter > 0) {
    for (const auto& [topic_name, data] : publishers_) {
      if (topic_name == "/camera/camera_info") {
        if (vo_calculator.K_received_) {
          continue; // Already received K, skip
        }
        // Deserialize the last message into a CameraInfo message
        auto camera_info_msg = deserializeMessage<sensor_msgs::msg::CameraInfo>(data.last_message);
        vo_calculator.set_K_from_CameraInfo(camera_info_msg);
      }
      // else if (topic_name == "/mavros/imu/data") {
      //   // Deserialize the last message into a IMU message
      //   auto imu_msg1 = deserializeMessage<sensor_msgs::msg::Imu>(prev_msg[topic_name]);
      //   vo_calculator.setCameraOrientation(imu_msg1->orientation);
      // }
      else if (topic_name == "/camera/image") {
        // Deserialize the last message into an Image message
        auto image_msg1 = deserializeMessage<sensor_msgs::msg::Image>(data.last_message);
        auto image_msg2 = deserializeMessage<sensor_msgs::msg::Image>(prev_msg[topic_name]);
        vo_calculator.calculate_T_with_frames(image_msg1, image_msg2);
      }
      else if (topic_name == "/simulation_pose_info") {
        // Deserialize the last message into a Pose message
        auto pose_msg2 = deserializeMessage<geometry_msgs::msg::PoseArray>(data.last_message);
        auto pose_msg1 = deserializeMessage<geometry_msgs::msg::PoseArray>(prev_msg[topic_name]);
        vo_calculator.calculate_gt_T(pose_msg1->poses[2], pose_msg2->poses[2]);
      } 
    }
    vo_calculator.visualize_matches();
    counter = 0;
  }

  else {
    counter++;
    for (const auto& [topic_name, data] : publishers_) {
      prev_msg[topic_name] = data.last_message;
    }
  }
  
}

void PlaybackPanel::onBrowseClicked()
{
  QString q_bag_directory = QFileDialog::getExistingDirectory(
    this,
    tr("Open ROS 2 Bag Directory"),
    QDir::homePath(),
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  
  if (q_bag_directory.isEmpty()) {
    return;
  }

  // Close any existing bag
  closeBag();
  
  // Open the new one
  openBag(q_bag_directory.toStdString());
}

void PlaybackPanel::openBag(const std::string& bag_filename)
{
  if (!rviz_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("PlaybackPanel"), "RViz node not initialized");
    file_path_editor_->setText("Error: RViz node not ready");
    return;
  }

  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename;
    // Use the factory to create the reader, just like your example
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);

    // Get all topics and types from the bag
    auto topics = reader_->get_all_topics_and_types();

    // Create a generic publisher for each topic
    for (const auto& topic_with_type : topics) {
      std::string topic_name = topic_with_type.name;
      std::string topic_type = topic_with_type.type;
      
      // Use "reliable" QoS with transient local durability for better compatibility
      rclcpp::QoS qos(10);
      qos.reliable();
      qos.transient_local();
      
      auto publisher = rviz_node_->create_generic_publisher(
        topic_name,
        topic_type,
        qos);
      
      // Initialize TopicData with publisher and null last_message
      TopicData data;
      data.publisher = publisher;
      data.last_message = nullptr;
      publishers_[topic_name] = data;
    }

    file_path_editor_->setText(QString::fromStdString(bag_filename));
    play_pause_button_->setEnabled(true);
    stop_button_->setEnabled(true);
    play_pause_button_->setText("Play");
    RCLCPP_INFO(rviz_node_->get_logger(), "Bag file loaded successfully: %s", bag_filename.c_str());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rviz_node_->get_logger(), "Failed to open bag file: %s", e.what());
    file_path_editor_->setText("Failed to load bag!");
  }
}

void PlaybackPanel::closeBag()
{
  // Stop timer first
  if (playback_timer_ && playback_timer_->isActive()) {
    playback_timer_->stop();
  }
  
  // Clear publishers before closing reader
  publishers_.clear();
  
  // Close and reset reader
  if (reader_) {
    reader_.reset();
  }
  
  play_pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  play_pause_button_->setText("Play");
}

void PlaybackPanel::onPlayPauseClicked()
{
  if (playback_timer_->isActive()) {
    playback_timer_->stop();
    play_pause_button_->setText("Play");
  } else {
    // Start the timer with the current spinbox value
    int interval_ms = timer_interval_spinbox_->value();
    playback_timer_->start(interval_ms);
    play_pause_button_->setText("Pause");
  }
}

void PlaybackPanel::onStopClicked()
{
  if (playback_timer_->isActive()) {
    playback_timer_->stop();
  }
  
  // "Stop" will reset the bag to the beginning by reloading it
  std::string current_bag = file_path_editor_->text().toStdString();
  
  // Only reopen if we have a valid bag path (not the default text)
  if (!current_bag.empty() && 
      current_bag != "Click Browse to select ROS 2 bag directory..." &&
      current_bag != "Failed to load bag!" &&
      current_bag != "Error: RViz node not ready") {
    closeBag();
    openBag(current_bag); // Re-open the bag to reset playback
  } else {
    // Just stop playback without reopening
    play_pause_button_->setText("Play");
  }
}

void PlaybackPanel::onTimerCallback()
{
  if (!reader_ || !reader_->has_next()) {
    playback_timer_->stop();
    play_pause_button_->setText("Play");
    RCLCPP_INFO(rviz_node_->get_logger(), "Bag playback finished.");
    return;
  }

  rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

  auto it = publishers_.find(msg->topic_name);
  if (it == publishers_.end()) {
    // This shouldn't happen if we pre-made all publishers
    return;
  }

  // Store the last read message
  it->second.last_message = msg;

  // Create a SerializedMessage object from the bag data
  rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

  // Publish the generic message
  it->second.publisher->publish(serialized_msg);
}

void PlaybackPanel::onTimerIntervalChanged(int value)
{
  // If the timer is currently running, restart it with the new interval
  if (playback_timer_->isActive()) {
    playback_timer_->start(value);
  }
}


// Load and save panel configuration (e.g., the last used bag file)
void PlaybackPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString filename;
  if (config.mapGetString("bag_filename", &filename)) {
    openBag(filename.toStdString());
  }
  int timer_interval;
  if (config.mapGetInt("timer_interval", &timer_interval)) {
    timer_interval_spinbox_->setValue(timer_interval);
  }
}

void PlaybackPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("bag_filename", file_path_editor_->text());
  config.mapSetValue("timer_interval", timer_interval_spinbox_->value());
}

} // namespace rviz_playback_panel

// Export the class as a plugin
PLUGINLIB_EXPORT_CLASS(rviz_playback_panel::PlaybackPanel, rviz_common::Panel)