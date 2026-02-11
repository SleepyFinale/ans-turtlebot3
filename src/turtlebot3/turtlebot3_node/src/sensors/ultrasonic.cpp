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

#include "turtlebot3_node/sensors/ultrasonic.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::Ultrasonic;

Ultrasonic::Ultrasonic(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & ultrasonic_topic_name)
: Sensors(nh)
{
  ultrasonic_pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>(ultrasonic_topic_name, this->qos_);

  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create ultrasonic publisher");
}

void Ultrasonic::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto ultrasonic_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

  ultrasonic_msg->header.stamp = now;

  ultrasonic_msg->angle_min = -1.57f;
  ultrasonic_msg->angle_max = 1.57f;
  ultrasonic_msg->angle_increment = 1.57f;
  ultrasonic_msg->time_increment = 0.0f;
  ultrasonic_msg->scan_time = 0.0f;
  ultrasonic_msg->range_min = 0.3f;
  ultrasonic_msg->range_max = 4.5f;

  ultrasonic_msg->ranges.resize(3);
  ultrasonic_msg->ranges[0] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_l.addr,
    extern_control_table.ultrasonic_l.length);

  ultrasonic_msg->ranges[1] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_f.addr,
    extern_control_table.ultrasonic_f.length);

  ultrasonic_msg->ranges[2] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_r.addr,
    extern_control_table.ultrasonic_r.length);

  ultrasonic_pub_->publish(std::move(ultrasonic_msg));
}
