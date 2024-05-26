/// \file
/// \brief This node intercepts laserscan messages and corrupts them to test slamtoolbox.
///
/// PARAMETERS:
///     \param window (int): The allowable range in degrees for the laserscan FOV
///
/// PUBLISHES:
///     \param /narrow/scan (sensor_msgs::msg::LaserScan): Publishes intercepted narrow laserscan
///
/// SUBSCRIBES:
///     \param /scan (sensor_msgs::msg::LaserScan): Subscribes to the laserscan for the turtlebot
#include <iostream>
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int64.hpp"
#include "visualization_msgs/msg/marker.hpp"


using namespace std::chrono_literals;

class Intercept : public rclcpp::Node
{
public:
  Intercept()
  : Node("intercept")
  {

    // Subscribers
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(
        &Intercept::laser_scan_callback, this,
        std::placeholders::_1));
    window_range_sub_ = create_subscription<std_msgs::msg::Int64>(
      "/fov", 10, std::bind(
        &Intercept::fov_callback, this,
        std::placeholders::_1));

    // Publishers
    updated_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/narrow/scan", 10);
    fov_range_publisher_ = create_publisher<visualization_msgs::msg::Marker>("/Marker", 10);
  }

private:
  // Create Objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr updated_scan_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr window_range_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_range_publisher_;
  // Variables
  int window = 360;

  /// \brief Laser Scan topic callback to narrow FOV
  void laser_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    // Set most things exactly as the original message
    sensor_msgs::msg::LaserScan new_scan;

    // Resize the ranges vector to match the original size
    new_scan.ranges.resize(msg.ranges.size());

    new_scan.header = msg.header;
    new_scan.angle_increment = msg.angle_increment;
    new_scan.angle_max = msg.angle_max;
    new_scan.angle_min = msg.angle_min;
    new_scan.range_max = msg.range_max;
    new_scan.range_min = msg.range_min;
    new_scan.time_increment = msg.time_increment;
    new_scan.angle_increment = msg.angle_increment;
    new_scan.scan_time = msg.scan_time;
    new_scan.intensities = msg.intensities;

    // Loop through and set 0 all out of range
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      if (i < static_cast<size_t>(window)) {
        new_scan.ranges.at(i) = msg.ranges.at(i);
      } else {
        new_scan.ranges.at(i) = 0.0;
      }
    }

    // Publish new reduced FOV laser scan
    updated_scan_publisher_->publish(new_scan);
  }

  /// \brief FOV topic callback to narrow FOV
  void fov_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    // Set window value
    window = static_cast<int>(msg->data);

    // Create and publish marker to see current value in RVIZ
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "text_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 3.0;
    marker.pose.position.y = 3.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 5.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.text = "FOV: " + std::to_string(window);
    fov_range_publisher_->publish(marker);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intercept>());
  rclcpp::shutdown();
  return 0;
}
