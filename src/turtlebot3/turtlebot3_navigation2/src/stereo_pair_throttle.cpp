// Republish time-matched stereo pairs at a low rate.
//
// The Arducams only offer 30 fps at 320x240, and the stereo consumers
// (stereo_depth_node: depth + visual odometry) use about 5 pairs per second.
// Receiving all 60 frames/s in Python cost more than half a core in rclpy's
// executor alone. This node takes the full streams in C++, keeps the last few
// frames per side, and publishes the closest left/right pair within slop_sec
// at most rate_hz times per second, stamps unchanged.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;

class StereoPairThrottle : public rclcpp::Node
{
public:
  StereoPairThrottle()
  : Node("stereo_pair_throttle")
  {
    rate_hz_ = declare_parameter<double>("rate_hz", 5.0);
    slop_ns_ = static_cast<int64_t>(declare_parameter<double>("slop_sec", 0.03) * 1e9);
    const auto left_in = declare_parameter<std::string>("left_in", "stereo_left/image_raw");
    const auto right_in = declare_parameter<std::string>("right_in", "stereo_right/image_raw");
    const auto left_out = declare_parameter<std::string>("left_out", "stereo_left/image_pair");
    const auto right_out = declare_parameter<std::string>("right_out", "stereo_right/image_pair");

    const auto qos = rclcpp::SensorDataQoS();
    left_pub_ = create_publisher<Image>(left_out, qos);
    right_pub_ = create_publisher<Image>(right_out, qos);
    left_sub_ = create_subscription<Image>(
      left_in, qos, [this](Image::ConstSharedPtr msg) {on_frame(msg, left_, right_);});
    right_sub_ = create_subscription<Image>(
      right_in, qos, [this](Image::ConstSharedPtr msg) {on_frame(msg, right_, left_);});
    RCLCPP_INFO(
      get_logger(), "stereo pairs %s + %s -> %s + %s at <= %.1f Hz",
      left_in.c_str(), right_in.c_str(), left_out.c_str(), right_out.c_str(), rate_hz_);
  }

private:
  static int64_t stamp_ns(const Image & msg)
  {
    return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  }

  void on_frame(
    Image::ConstSharedPtr msg, std::deque<Image::ConstSharedPtr> & mine,
    std::deque<Image::ConstSharedPtr> & other)
  {
    mine.push_back(msg);
    if (mine.size() > 3) {
      mine.pop_front();
    }
    const auto now = std::chrono::steady_clock::now();
    if (rate_hz_ > 0.0 &&
      now - last_sent_ < std::chrono::duration<double>(1.0 / rate_hz_))
    {
      return;
    }
    // The newest frame against every buffered frame from the other camera.
    Image::ConstSharedPtr best;
    int64_t best_gap = slop_ns_ + 1;
    for (const auto & candidate : other) {
      const int64_t gap = std::llabs(stamp_ns(*candidate) - stamp_ns(*msg));
      if (gap < best_gap) {
        best_gap = gap;
        best = candidate;
      }
    }
    if (!best) {
      return;
    }
    const bool msg_is_left = (&mine == &left_);
    left_pub_->publish(*(msg_is_left ? msg : best));
    right_pub_->publish(*(msg_is_left ? best : msg));
    last_sent_ = now;
    left_.clear();
    right_.clear();
  }

  double rate_hz_;
  int64_t slop_ns_;
  std::deque<Image::ConstSharedPtr> left_;
  std::deque<Image::ConstSharedPtr> right_;
  std::chrono::steady_clock::time_point last_sent_{};
  rclcpp::Publisher<Image>::SharedPtr left_pub_;
  rclcpp::Publisher<Image>::SharedPtr right_pub_;
  rclcpp::Subscription<Image>::SharedPtr left_sub_;
  rclcpp::Subscription<Image>::SharedPtr right_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoPairThrottle>());
  rclcpp::shutdown();
  return 0;
}
