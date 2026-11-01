#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace daq_bringup
{
class RadarPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit RadarPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void refresh();
  QLabel * status_;
  QLabel * frame_;
  QLabel * origin_;
  QTableWidget * table_;
  QTimer timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::mutex mutex_;
  std::string latest_;
  bool dirty_{false};
  bool received_{false};
  std::chrono::steady_clock::time_point last_received_;
};
}  // namespace daq_bringup
