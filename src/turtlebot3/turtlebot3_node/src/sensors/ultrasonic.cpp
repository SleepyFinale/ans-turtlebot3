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
#include <cmath>


using robotis::turtlebot3::sensors::Ultrasonic;

Ultrasonic::Ultrasonic(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & ultrasonic_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  ultrasonic_left_pub_ = nh->create_publisher<sensor_msgs::msg::Range>(ultrasonic_topic_name+"_l", this->qos_);
  ultrasonic_front_pub_ = nh->create_publisher<sensor_msgs::msg::Range>(ultrasonic_topic_name+"_f", this->qos_);
  ultrasonic_right_pub_ = nh->create_publisher<sensor_msgs::msg::Range>(ultrasonic_topic_name+"_r", this->qos_);

  nh_->declare_parameter<float>("ultrasonic.min_valid_range", min_valid_range_);
  nh_->declare_parameter<float>("ultrasonic.max_valid_range", max_valid_range_);
  nh_->declare_parameter<bool>("ultrasonic.use_jump_filter", use_jump_filter_);
  nh_->declare_parameter<float>("ultrasonic.max_delta_per_cycle", max_delta_per_cycle_);
  nh_->declare_parameter<int>("ultrasonic.max_hold_cycles", max_hold_cycles_);
  nh_->declare_parameter<float>("ultrasonic.release_step_per_cycle", release_step_per_cycle_);
  nh_->declare_parameter<float>("ultrasonic.left_offset", range_offsets_[0]);
  nh_->declare_parameter<float>("ultrasonic.front_offset", range_offsets_[1]);
  nh_->declare_parameter<float>("ultrasonic.right_offset", range_offsets_[2]);

  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));
  nh_->get_parameter_or<float>("ultrasonic.min_valid_range", min_valid_range_, min_valid_range_);
  nh_->get_parameter_or<float>("ultrasonic.max_valid_range", max_valid_range_, max_valid_range_);
  nh_->get_parameter_or<bool>("ultrasonic.use_jump_filter", use_jump_filter_, use_jump_filter_);
  nh_->get_parameter_or<float>("ultrasonic.max_delta_per_cycle", max_delta_per_cycle_, max_delta_per_cycle_);
  nh_->get_parameter_or<int>("ultrasonic.max_hold_cycles", max_hold_cycles_, max_hold_cycles_);
  nh_->get_parameter_or<float>("ultrasonic.release_step_per_cycle", release_step_per_cycle_, release_step_per_cycle_);
  nh_->get_parameter_or<float>("ultrasonic.left_offset", range_offsets_[0], range_offsets_[0]);
  nh_->get_parameter_or<float>("ultrasonic.front_offset", range_offsets_[1], range_offsets_[1]);
  nh_->get_parameter_or<float>("ultrasonic.right_offset", range_offsets_[2], range_offsets_[2]);

  if (min_valid_range_ < 0.0f) {
    min_valid_range_ = 0.0f;
  }
  if (max_valid_range_ <= min_valid_range_) {
    max_valid_range_ = 3.0f;
  }
  if (max_hold_cycles_ < 0) {
    max_hold_cycles_ = 0;
  }
  if (release_step_per_cycle_ <= 0.0f) {
    release_step_per_cycle_ = 0.05f;
  }

  if (name_space_ != "") {
    frame_id_ = name_space_ + "/" + frame_id_;
  }
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create ultrasonic publisher");
}


void Ultrasonic::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  float raw_dist[3];
  std::array<float, 3> filtered_dist;
  const float coneAngle = 15 * (3.145926/180);

  raw_dist[2] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_l.addr,
    extern_control_table.ultrasonic_l.length);
  raw_dist[1] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_f.addr,
    extern_control_table.ultrasonic_f.length);
  raw_dist[0] = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.ultrasonic_r.addr,
    extern_control_table.ultrasonic_r.length);

  for (size_t i = 0; i < filtered_dist.size(); ++i) {
    float dist = raw_dist[i] + range_offsets_[i];
    bool valid = std::isfinite(dist) && dist >= min_valid_range_ && dist <= max_valid_range_;

    if (valid && use_jump_filter_ && previous_ranges_[i] > 0.0f) {
      const float prev = previous_ranges_[i];
      const float delta = dist - prev;  // negative means "closer" (fast approach)

      // Reject one-tick *increases* that are too large (often multipath/echoes),
      // but never reject large *decreases* — that is the normal signature of
      // a suddenly appearing close obstacle in front of the robot.
      if (delta > max_delta_per_cycle_) {
        valid = false;
      }
    }

    if (!valid) {
      // Hold a previously valid close reading for a bounded number of cycles,
      // then gradually release toward max range to avoid permanent "ghost"
      // obstacles when the sensor stream becomes sticky/noisy.
      if (previous_ranges_[i] > 0.0f) {
        hold_cycles_[i] += 1;
        if (hold_cycles_[i] <= max_hold_cycles_) {
          filtered_dist[i] = previous_ranges_[i];
        } else {
          previous_ranges_[i] = std::min(
            previous_ranges_[i] + release_step_per_cycle_,
            max_valid_range_);
          filtered_dist[i] = previous_ranges_[i];
        }
      } else {
        filtered_dist[i] = max_valid_range_;
      }
    } else {
      hold_cycles_[i] = 0;
      filtered_dist[i] = dist;
      previous_ranges_[i] = dist;
    }
  }

  auto make_range_msg = [&](float dist, const std::string & msg_frame_id)
  {
    sensor_msgs::msg::Range msg;

    msg.header.stamp = now;
    msg.header.frame_id = msg_frame_id;

    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

    msg.field_of_view = coneAngle;

    msg.min_range = min_valid_range_;
    msg.max_range = max_valid_range_;

    // Handle invalid readings
    if (dist <= msg.min_range || dist > msg.max_range)
    {
      msg.range = msg.max_range;  // treat as "no obstacle"
    }
    else
    {
      msg.range = dist;
    }

    return msg;
  };

  ultrasonic_left_pub_->publish(
    make_range_msg(filtered_dist[2], frame_id_ + "_left"));

  ultrasonic_front_pub_->publish(
    make_range_msg(filtered_dist[1], frame_id_ + "_front"));

  ultrasonic_right_pub_->publish(
    make_range_msg(filtered_dist[0], frame_id_ + "_right"));

}

// void Ultrasonic::publish(
//   const rclcpp::Time & now,
//   std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
// {
//   auto ultrasonic_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

//   const float senpos[3] = {-0.78,0.0,0.78};
//   const float angle_min = -(3.145926/2); //senpos[0] - 0.52f;
//   const float angle_max = (3.145926/2); //senpos[2] + 0.52f;
//   const float angle_increment = 0.017f;
//   const float coneAngle = 5 * (3.145926/180);
//   int numPoints = (int)((angle_max - angle_min)/angle_increment);
//   float sdist[3];

//   int aIndex, senToUse;
//   ultrasonic_msg->header.frame_id = this->frame_id_;
//   ultrasonic_msg->header.stamp = now;


//   ultrasonic_msg->angle_min = angle_min;
//   ultrasonic_msg->angle_max = angle_max;
//   ultrasonic_msg->angle_increment = angle_increment;
//   ultrasonic_msg->time_increment = 0.0f;
//   ultrasonic_msg->scan_time = 0.0f;
//   ultrasonic_msg->range_min = 0.03f;
//   ultrasonic_msg->range_max = 3.0f;

 
//   sdist[2] = dxl_sdk_wrapper->get_data_from_device<float>(
//     extern_control_table.ultrasonic_l.addr,
//     extern_control_table.ultrasonic_l.length) + 0.095;
//   sdist[1] = dxl_sdk_wrapper->get_data_from_device<float>(
//     extern_control_table.ultrasonic_f.addr,
//     extern_control_table.ultrasonic_f.length) + 0.1;
//   sdist[0] = dxl_sdk_wrapper->get_data_from_device<float>(
//     extern_control_table.ultrasonic_r.addr,
//     extern_control_table.ultrasonic_r.length) + 0.095;
  
//   ultrasonic_msg->ranges.resize(numPoints);
//   ultrasonic_msg->ranges[0] = std::numeric_limits<float>::infinity();
//   for (aIndex = 1; aIndex < numPoints; aIndex++)
//   {
//     senToUse = ((float)aIndex/numPoints)*3;
//     if ((angle_min + angle_increment * aIndex <= senpos[senToUse] + coneAngle) && 
//     (angle_min + angle_increment * aIndex >= senpos[senToUse] - coneAngle)) 
//     {
//       ultrasonic_msg->ranges[aIndex] = sdist[senToUse];
//     } else {
//       ultrasonic_msg->ranges[aIndex] = std::numeric_limits<float>::infinity();
//     }
//   }

//   ultrasonic_pub_->publish(std::move(ultrasonic_msg));
// }
