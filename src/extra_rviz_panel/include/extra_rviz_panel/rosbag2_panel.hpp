#ifndef PLAYBACK_PANEL_HPP_
#define PLAYBACK_PANEL_HPP_

#include <map>
#include <memory>
#include <string>

#include <QTimer>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <QComboBox>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

namespace rviz_playback_panel
{

class PlaybackPanel : public rviz_common::Panel
{
  // This macro is required for any QObject that has custom signals or slots
  Q_OBJECT

public:
  // The constructor must pass the parent widget to the base class
  explicit PlaybackPanel(QWidget* parent = nullptr);
  virtual ~PlaybackPanel();

  // Called after the panel is added to the display
  virtual void onInitialize();

  // Load and save configuration data
  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

protected:
  // Qt slots are functions that can be connected to signals (like button clicks)
protected Q_SLOTS:
  void onBrowseClicked();
  void onPlayPauseClicked();
  void onStopClicked();
  void onTimerCallback(); // This will be our main playback loop
  void onTimerIntervalChanged(int value);
  void onChooseMomentClicked();
  void onMethodChanged(int index);

private:
  void openBag(const std::string& bag_filename);
  void closeBag();

  // --- Qt UI Elements ---
  QLineEdit* file_path_editor_;
  QPushButton* browse_button_;
  QPushButton* play_pause_button_;
  QPushButton* stop_button_;
  QPushButton* choose_moment_button_;
  QSpinBox* timer_interval_spinbox_;
  QLabel* timer_interval_label_;
  QComboBox* method_selector_;
  QLabel* method_label_;

  // --- ROS & Bag Elements ---
  // We use a QTimer for GUI-safe ticking
  QTimer* playback_timer_;
  
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  // Structure to hold both publisher and last message for each topic
  struct TopicData {
    rclcpp::GenericPublisher::SharedPtr publisher;
    rosbag2_storage::SerializedBagMessageSharedPtr last_message;
  };

  // A map to hold generic publishers and last message for each topic in the bag
  // Key: topic_name, Value: TopicData (publisher + last message)
  std::map<std::string, TopicData> publishers_;

  // Node context for creating publishers
  rclcpp::Node::SharedPtr rviz_node_;
};

} // namespace rviz_playback_panel

#endif // PLAYBACK_PANEL_HPP_