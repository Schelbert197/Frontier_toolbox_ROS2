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


using namespace std::chrono_literals;

class Intercept : public rclcpp::Node
{
public:
  Intercept()
  : Node("intercept")
  {
    // Parameter to define size of lidar range
    // auto window_des = rcl_interfaces::msg::ParameterDescriptor{};
    // window_des.description = "Timer callback window [Hz] (default 100 Hz)";
    // declare_parameter("window", 60, window_des);
    // int window = get_parameter("window").get_parameter_value().get<int>();

    // Subscribers
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(
        &Intercept::laser_scan_callback, this,
        std::placeholders::_1));

    // Publishers
    updated_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/narrow/scan", 10);
  }

private:
  // Create Objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr updated_scan_publisher_;
  // Variables
  int window = 60;

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

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intercept>());
  rclcpp::shutdown();
  return 0;
}
