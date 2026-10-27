// Copy /<robot>/tf onto /<robot>/tf_zenoh without stalling SLAM.
//
// C++ port of scripts/comms/tf_zenoh_publish.py, which cost about a fifth of
// a core on the Pi just receiving ~50 Hz of /tf in rclpy.
//
// zenoh-bridge-ros2dds subscribes with the publisher's QoS. A reliable
// subscription on /<robot>/tf blocks slam_toolbox, so map->odom freezes and
// Nav2 reports "Transform data too old". This node subscribes best-effort
// (no backpressure) and republishes a throttled copy for the bridge.
//
// /<robot>/tf has several publishers (SLAM, EKF, robot_state_publisher). Each
// message holds only that publisher's edges. The bridge keeps one sample, so
// every tick this node sends the edges that changed since the last tick in
// one message. Unchanged edges are not resent: a repeated stamp makes tf2 on
// the central PC log TF_REPEATED_DATA.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;
using Edge = std::pair<std::string, std::string>;

namespace
{
int64_t stamp_ns(const TransformStamped & tf)
{
  return static_cast<int64_t>(tf.header.stamp.sec) * 1000000000LL + tf.header.stamp.nanosec;
}

void store(std::map<Edge, TransformStamped> & bucket, const TFMessage & msg)
{
  for (const auto & tf : msg.transforms) {
    if (!tf.header.frame_id.empty() && !tf.child_frame_id.empty()) {
      bucket[{tf.header.frame_id, tf.child_frame_id}] = tf;
    }
  }
}
}  // namespace

class TfZenohRelay : public rclcpp::Node
{
public:
  explicit TfZenohRelay(const std::string & robot)
  : Node("tf_zenoh_publish")
  {
    const auto best_effort_in = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
    const auto best_effort_out = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto latched_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto best_effort_latched_out =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    tf_sub_ = create_subscription<TFMessage>(
      "/" + robot + "/tf", best_effort_in,
      [this](TFMessage::ConstSharedPtr msg) {store(tf_, *msg);});
    static_sub_ = create_subscription<TFMessage>(
      "/" + robot + "/tf_static", latched_in,
      [this](TFMessage::ConstSharedPtr msg) {
        store(tf_static_, *msg);
        tick_static();
      });
    tf_pub_ = create_publisher<TFMessage>("/" + robot + "/tf_zenoh", best_effort_out);
    static_pub_ = create_publisher<TFMessage>(
      "/" + robot + "/tf_static_zenoh", best_effort_latched_out);

    using namespace std::chrono_literals;
    tf_timer_ = create_wall_timer(100ms, [this]() {tick_tf();});
    // Static edges are resent whole every second so a central subscriber
    // that joins late still gets them over the best-effort route.
    static_timer_ = create_wall_timer(1s, [this]() {tick_static();});
    log_timer_ = create_wall_timer(30s, [this]() {tick_log();});
    // start_fleet.sh waits for this exact line.
    RCLCPP_INFO(
      get_logger(),
      "TF zenoh publish: /%s/tf -> /%s/tf_zenoh (best-effort in, changed edges only, 10 Hz)",
      robot.c_str(), robot.c_str());
  }

private:
  void tick_tf()
  {
    TFMessage out;
    for (const auto & [key, tf] : tf_) {
      const int64_t stamp = stamp_ns(tf);
      auto it = sent_.find(key);
      if (it != sent_.end() && it->second == stamp) {
        continue;
      }
      sent_[key] = stamp;
      out.transforms.push_back(tf);
    }
    if (!out.transforms.empty()) {
      tf_pub_->publish(out);
    }
  }

  void tick_static()
  {
    if (tf_static_.empty()) {
      return;
    }
    TFMessage out;
    out.transforms.reserve(tf_static_.size());
    for (const auto & entry : tf_static_) {
      out.transforms.push_back(entry.second);
    }
    static_pub_->publish(out);
  }

  std::string edge_age(const std::string & child_suffix) const
  {
    const double now = this->now().seconds();
    for (const auto & [key, tf] : tf_) {
      const std::string & child = key.second;
      const bool match = child == child_suffix ||
        (child.size() > child_suffix.size() &&
        child.compare(child.size() - child_suffix.size() - 1, std::string::npos,
        "/" + child_suffix) == 0);
      if (match) {
        char buf[256];
        std::snprintf(
          buf, sizeof(buf), "%s->%s age=%.2fs", key.first.c_str(), child.c_str(),
          now - static_cast<double>(stamp_ns(tf)) * 1e-9);
        return buf;
      }
    }
    return "missing";
  }

  void tick_log()
  {
    if (tf_.empty() && tf_static_.empty()) {
      RCLCPP_WARN(get_logger(), "No TF received yet on /tf or /tf_static");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "TF %s; %s; static edges: %zu",
      edge_age("odom").c_str(), edge_age("base_footprint").c_str(), tf_static_.size());
  }

  // Everything runs on the single-threaded executor, so no locking.
  std::map<Edge, TransformStamped> tf_;
  std::map<Edge, TransformStamped> tf_static_;
  std::map<Edge, int64_t> sent_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<TFMessage>::SharedPtr static_sub_;
  rclcpp::Publisher<TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr static_pub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr static_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() != 2 || args[1].empty()) {
    std::fprintf(stderr, "usage: tf_zenoh_relay <robot>\n");
    rclcpp::shutdown();
    return 2;
  }
  std::string robot = args[1];
  while (!robot.empty() && robot.front() == '/') {robot.erase(0, 1);}
  while (!robot.empty() && robot.back() == '/') {robot.pop_back();}
  rclcpp::spin(std::make_shared<TfZenohRelay>(robot));
  rclcpp::shutdown();
  return 0;
}
