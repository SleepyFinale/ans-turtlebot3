// The only publisher of /<robot>/cmd_vel while Nav2 runs.
//
// Several nodes drive the base: the controller (through the velocity
// smoother), Nav2's spin/backup behaviors, stuck_detector's reflexes and the
// startup map seeder. When each published cmd_vel itself the base obeyed
// whichever message came last, so a 40 Hz reflex backing away and the 10 Hz
// controller still pushing forward alternated every few milliseconds: the
// robot shook in place instead of backing up. The stereo and ultrasonic stop
// gates did the same with zero twists, so a stop also came out as a stutter.
//
// Here each source has its own topic and a priority. The freshest
// high-priority source wins until it has been silent for timeout_sec; lower
// ones are dropped meanwhile. Stop gates are Bool topics: while any of them
// reported true within gate_timeout_sec, forward speed from Nav2 is clamped
// to zero (turning and reversing still pass, so the controller and behaviors
// can steer away). Reflexes are not gated: they check their own path against
// the LiDAR and the low-obstacle memory, and a tilt or clearance escape must
// be able to move.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using geometry_msgs::msg::Twist;
using std_msgs::msg::Bool;

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux()
  : Node("cmd_vel_mux")
  {
    // Highest priority first.
    const auto inputs = declare_parameter<std::vector<std::string>>(
      "inputs", {"cmd_vel_reflex", "cmd_vel_behavior", "cmd_vel_smoothed"});
    const auto gated = declare_parameter<std::vector<std::string>>(
      "gated_inputs", {"cmd_vel_behavior", "cmd_vel_smoothed"});
    const auto gates = declare_parameter<std::vector<std::string>>(
      "gates", {"stereo/forward_block", "ultrasonic/forward_block"});
    // Nav2 publishes at 10 Hz; three missed messages hand control down.
    timeout_ = declare_parameter<double>("timeout_sec", 0.3);
    gate_timeout_ = declare_parameter<double>("gate_timeout_sec", 0.3);

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_ = create_publisher<Twist>(declare_parameter<std::string>("output", "cmd_vel"), reliable);
    active_pub_ = create_publisher<std_msgs::msg::String>(
      "cmd_vel_mux/active", rclcpp::QoS(1).reliable().transient_local());

    sources_.resize(inputs.size());
    for (size_t i = 0; i < inputs.size(); ++i) {
      sources_[i].name = inputs[i];
      sources_[i].gated = std::find(gated.begin(), gated.end(), inputs[i]) != gated.end();
      subs_.push_back(create_subscription<Twist>(
        inputs[i], reliable, [this, i](Twist::ConstSharedPtr msg) {on_input(i, *msg);}));
    }
    gate_until_.assign(gates.size(), rclcpp::Time(0, 0, RCL_STEADY_TIME));
    for (size_t i = 0; i < gates.size(); ++i) {
      gate_subs_.push_back(create_subscription<Bool>(
        gates[i], 10, [this, i](Bool::ConstSharedPtr msg) {
          gate_until_[i] = msg->data ?
          steady_.now() + rclcpp::Duration::from_seconds(gate_timeout_) :
          rclcpp::Time(0, 0, RCL_STEADY_TIME);
        }));
    }
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(50ms, [this]() {tick();});
    publish_active(-1);
    RCLCPP_INFO(get_logger(), "cmd_vel mux: %zu inputs, %zu stop gates", inputs.size(), gates.size());
  }

private:
  struct Source
  {
    std::string name;
    bool gated{false};
    rclcpp::Time last{0, 0, RCL_STEADY_TIME};
  };

  bool fresh(const Source & s, const rclcpp::Time & now) const
  {
    return s.last.nanoseconds() > 0 && (now - s.last).seconds() <= timeout_;
  }

  int winner(const rclcpp::Time & now) const
  {
    for (size_t i = 0; i < sources_.size(); ++i) {
      if (fresh(sources_[i], now)) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  bool gate_closed(const rclcpp::Time & now) const
  {
    return std::any_of(
      gate_until_.begin(), gate_until_.end(),
      [&now](const rclcpp::Time & t) {return t > now;});
  }

  void on_input(size_t index, const Twist & msg)
  {
    const auto now = steady_.now();
    sources_[index].last = now;
    const int win = winner(now);
    if (win != static_cast<int>(index)) {
      return;  // a higher-priority source is driving
    }
    Twist out = msg;
    if (sources_[index].gated && out.linear.x > 0.0 && gate_closed(now)) {
      out.linear.x = 0.0;
    }
    pub_->publish(out);
    stopped_ = out.linear.x == 0.0 && out.angular.z == 0.0;
    publish_active(win);
  }

  void tick()
  {
    const int win = winner(steady_.now());
    if (win < 0 && !stopped_) {
      // Every source went quiet mid-motion: stop rather than leave the last
      // command to the base's own timeout.
      pub_->publish(Twist());
      stopped_ = true;
    }
    publish_active(win);
  }

  void publish_active(int win)
  {
    if (win == active_) {
      return;
    }
    active_ = win;
    std_msgs::msg::String msg;
    msg.data = win < 0 ? "none" : sources_[win].name;
    active_pub_->publish(msg);
  }

  rclcpp::Clock steady_{RCL_STEADY_TIME};
  double timeout_{0.3};
  double gate_timeout_{0.3};
  std::vector<Source> sources_;
  std::vector<rclcpp::Time> gate_until_;
  bool stopped_{true};
  int active_{-2};
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  std::vector<rclcpp::Subscription<Twist>::SharedPtr> subs_;
  std::vector<rclcpp::Subscription<Bool>::SharedPtr> gate_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMux>());
  rclcpp::shutdown();
  return 0;
}
