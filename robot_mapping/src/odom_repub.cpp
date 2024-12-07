// Copyright 2024 Srikanth Schelbert.
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

/// @file
/// @brief A a node to simply republish a topic

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomRepub : public rclcpp::Node
{
public:
  OdomRepub()
  : Node("odom_repub")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/platform/odom/filtered", 10, std::bind(&OdomRepub::odom_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

private:
// Create Objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Save Odometry 
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.pose = msg->pose;
    odom.twist = msg->twist;
    odom.child_frame_id = msg->child_frame_id;

    // Create and publish path to odom frame
    odom_pub_->publish(odom);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomRepub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}