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
  const std::string & ultrasonic_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  ultrasonic_pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>(ultrasonic_topic_name, this->qos_);

  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));

  if (name_space_ != "") {
    frame_id_ = name_space_ + "/" + frame_id_;
  }
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create ultrasonic publisher");
}

void Ultrasonic::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto ultrasonic_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

  const float senpos[3] = {-0.78,0.0,0.78};
  const float angle_min = senpos[0] - 0.52f;
  const float angle_max = senpos[2] + 0.52f;
  const float angle_increment = 0.017f;
  const float coneAngle = 5 * (3.145926/180);
  int numPoints = (int)((angle_max - angle_min)/angle_increment);
  float sdist[3];

  int aIndex, senToUse;
  ultrasonic_msg->header.frame_id = this->frame_id_;
  ultrasonic_msg->header.stamp = now;


  ultrasonic_msg->angle_min = angle_min;
  ultrasonic_msg->angle_max = angle_max;
  ultrasonic_msg->angle_increment = angle_increment;
  ultrasonic_msg->time_increment = 0.0f;
  ultrasonic_msg->scan_time = 0.0f;
  ultrasonic_msg->range_min = 0.03f;
  ultrasonic_msg->range_max = 4.5f;

 
  sdist[2] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_l.addr,
    extern_control_table.ultrasonic_l.length) + 0.095;
  sdist[1] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_f.addr,
    extern_control_table.ultrasonic_f.length) + 0.1;
  sdist[0] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_r.addr,
    extern_control_table.ultrasonic_r.length) + 0.095;
  
  ultrasonic_msg->ranges.resize(numPoints);
  ultrasonic_msg->ranges[0] = -1;
  for (aIndex = 1; aIndex < numPoints; aIndex++)
  {
    senToUse = ((float)aIndex/numPoints)*3;
    if ((angle_min + angle_increment * aIndex <= senpos[senToUse] + coneAngle) && 
    (angle_min + angle_increment * aIndex >= senpos[senToUse] - coneAngle)) 
    {
      ultrasonic_msg->ranges[aIndex] = sdist[senToUse];
    } else {
      ultrasonic_msg->ranges[aIndex] = -1;
    }
  }

  ultrasonic_pub_->publish(std::move(ultrasonic_msg));
}
