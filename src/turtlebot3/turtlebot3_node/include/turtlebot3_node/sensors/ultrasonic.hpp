// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#ifndef TURTLEBOT3_NODE__SENSORS__ULTRASONIC_HPP_
#define TURTLEBOT3_NODE__SENSORS__ULTRASONIC_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/range.hpp>


#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class Ultrasonic : public Sensors
{
public:
  explicit Ultrasonic(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & ultrasonic_topic_name = "ultrasonic",
    const std::string & frame_id = "ultrasonic_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_right_pub_;

  std::string name_space_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__IMU_HPP_
