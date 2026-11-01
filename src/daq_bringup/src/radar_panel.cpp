#include "daq_bringup/radar_panel.hpp"

#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace daq_bringup
{
RadarPanel::RadarPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  status_ = new QLabel("Waiting for radar scans", this);
  frame_ = new QLabel("Coordinates: waiting for mounting configuration", this);
  origin_ = new QLabel("Radar origin: waiting", this);
  for (auto * label : {status_, frame_, origin_}) {
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setTextFormat(Qt::PlainText);
    layout->addWidget(label);
  }
  table_ = new QTableWidget(0, 6, this);
  table_->setHorizontalHeaderLabels({"#", "X [m]", "Y [m]", "Z [m]", "R [m]", "SNR [dB]"});
  table_->verticalHeader()->hide();
  table_->verticalHeader()->setDefaultSectionSize(24);
  table_->setMinimumHeight(340);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setAlternatingRowColors(true);
  table_->setToolTip("XYZ: mounting-parent coordinates. R: distance from radar origin.\n"
    "#: index in the current scan, not a tracked target ID. Missing SNR is N/A.");
  layout->addWidget(table_);
  auto * note = new QLabel("# = current scan index (no tracking)\n"
    "Mount pose only; world position is not estimated.", this);
  note->setWordWrap(true);
  layout->addWidget(note);
  setMinimumWidth(430);
  connect(&timer_, &QTimer::timeout, this, &RadarPanel::refresh);
  timer_.start(100);
}

void RadarPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_->setText("RViz ROS node unavailable");
    return;
  }
  subscription_ = abstraction->get_raw_node()->create_subscription<std_msgs::msg::String>(
    "/daq/view/radar/report", rclcpp::QoS(1),
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = message->data;
      dirty_ = received_ = true;
      last_received_ = std::chrono::steady_clock::now();
    });
}

void RadarPanel::refresh()
{
  std::string payload;
  bool stale;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stale = received_ && std::chrono::steady_clock::now() - last_received_ >
      std::chrono::milliseconds(500);
    if (dirty_) {
      payload.swap(latest_);
      dirty_ = false;
    }
  }
  if (!payload.empty()) {
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(payload), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject()) {
      status_->setText("Invalid radar report");
      table_->setRowCount(0);
      return;
    }
    const auto report = document.object();
    const auto frame = report["frame"].toString();
    frame_->setText("XYZ frame: " + frame + (frame == "camera_link" ?
      "\nX forward / Y left / Z up" : "\nAxes follow the configured parent frame"));
    const auto xyz = report["sensor_xyz_m"].toArray();
    const auto q = report["sensor_quaternion_xyzw"].toArray();
    if (xyz.size() == 3 && q.size() == 4) {
      origin_->setText(QString("Radar origin in %1 [m]:\n(%2, %3, %4)\nMount: %5")
        .arg(frame).arg(xyz[0].toDouble(), 0, 'f', 3).arg(xyz[1].toDouble(), 0, 'f', 3)
        .arg(xyz[2].toDouble(), 0, 'f', 3)
        .arg(report["calibrated"].toBool() ? "calibrated" : "ESTIMATE (uncalibrated)"));
      origin_->setToolTip(QString("%1 -> %2\nQuaternion xyzw: (%3, %4, %5, %6)")
        .arg(frame, report["sensor_frame"].toString())
        .arg(q[0].toDouble()).arg(q[1].toDouble()).arg(q[2].toDouble()).arg(q[3].toDouble()));
    }
    const auto points = report["points"].toArray();
    const bool live = report["state"].toString() == "live" && !stale;
    table_->setRowCount(live ? points.size() : 0);
    for (int row = 0; row < table_->rowCount(); ++row) {
      const auto point = points[row].toObject();
      const auto p = point["xyz_m"].toArray();
      if (p.size() != 3) {continue;}
      const QStringList values = {QString::number(point["id"].toInt()),
        QString::number(p[0].toDouble(), 'f', 3), QString::number(p[1].toDouble(), 'f', 3),
        QString::number(p[2].toDouble(), 'f', 3), QString::number(point["range_m"].toDouble(), 'f', 3),
        point["snr_db"].isDouble() ? QString::number(point["snr_db"].toDouble(), 'f', 1) : "N/A"};
      for (int column = 0; column < values.size(); ++column) {
        auto * item = table_->item(row, column);
        if (!item) {
          item = new QTableWidgetItem();
          item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
          table_->setItem(row, column, item);
        }
        item->setText(values[column]);
      }
    }
    status_->setText(report["state"].toString() == "waiting" ? "Waiting for first radar scan" :
      live ? QString("LIVE | %1 points | host stamp %2 s")
      .arg(points.size()).arg(report["stamp_ns"].toDouble() / 1e9, 0, 'f', 3) :
      "NO FRESH SCAN (>0.5 s) | previous points cleared");
  } else if (stale) {
    table_->setRowCount(0);
    status_->setText("NO FRESH SCAN (>0.5 s) | previous points cleared");
  }
}
}  // namespace daq_bringup

PLUGINLIB_EXPORT_CLASS(daq_bringup::RadarPanel, rviz_common::Panel)
