#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageRateThrottle : public rclcpp::Node
{
public:
  ImageRateThrottle()
  : Node("image_rate_throttle")
  {
    const double max_rate_hz = declare_parameter<double>("max_rate_hz", 15.0);
    if (!std::isfinite(max_rate_hz) || max_rate_hz <= 0.0) {
      throw std::invalid_argument("max_rate_hz must be a finite positive value");
    }

    period_ns_ = static_cast<int64_t>(std::llround(1.0e9 / max_rate_hz));
    tolerance_ns_ = std::min<int64_t>(2000000, period_ns_ / 20);

    // RealSense sensor images are normally Best Effort. The standalone
    // depth_image_proc register node in Humble subscribes with Reliable QoS,
    // so bridge the two policies at this visualization-only boundary.
    const auto input_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    publisher_ = create_publisher<sensor_msgs::msg::Image>("output", output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "input", input_qos,
      [this](sensor_msgs::msg::Image::UniquePtr message) {
        throttle(std::move(message));
      });

    RCLCPP_INFO(get_logger(), "Limiting image stream to %.3f Hz", max_rate_hz);
  }

private:
  void throttle(sensor_msgs::msg::Image::UniquePtr message)
  {
    int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp_ns <= 0) {
      stamp_ns = now().nanoseconds();
    }

    // Reset the phase after a clock reset or a long stream interruption.
    if (next_publish_ns_ == 0 || stamp_ns + period_ns_ < next_publish_ns_ ||
      stamp_ns > next_publish_ns_ + 4 * period_ns_)
    {
      next_publish_ns_ = stamp_ns;
    }

    if (stamp_ns + tolerance_ns_ < next_publish_ns_) {
      return;
    }

    publisher_->publish(std::move(message));
    do {
      next_publish_ns_ += period_ns_;
    } while (next_publish_ns_ <= stamp_ns + tolerance_ns_);
  }

  int64_t period_ns_{0};
  int64_t tolerance_ns_{0};
  int64_t next_publish_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImageRateThrottle>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("image_rate_throttle"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
