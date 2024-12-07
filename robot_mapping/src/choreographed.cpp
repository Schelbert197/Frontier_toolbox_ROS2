#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "visualization_msgs/msg/marker.hpp"

class Choreographed : public rclcpp::Node
{
public:
  Choreographed()
  : Node("choreographed"), counter_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    error_publisher_ = create_publisher<visualization_msgs::msg::Marker>("/error_marker", 10);
    error_sub_ =
      this->create_subscription<std_msgs::msg::Float32>(
      "/error", 10,
      std::bind(&Choreographed::error_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 200Hz -> 1000ms / 100 = 10ms
      std::bind(&Choreographed::timer_callback, this)
    );
  }

private:
  // Objects
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr error_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Variables
  int counter_ = -100;
  double current_error_ = 0.0;

  /// @brief Listener on error topic saves current error
  /// @param err
  void error_callback(const std_msgs::msg::Float32 err)
  {
    current_error_ = err.data;
  }

  void publish_error()
  {
    // Create and publish marker to see current value in RVIZ
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "text_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 3.0;
    marker.pose.position.y = -2.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 4.0;
    marker.scale.z = 0.8;
    marker.color.r = 0.8f;
    marker.color.g = 0.2f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.text = "Error: " + std::to_string(current_error_);
    error_publisher_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Total Aggregated and Normalized error: %f", current_error_);
  }

  /// @brief Creates the Twist msg object and publishes it
  /// @param x_vec desired speed in x-dir
  /// @param rot_vec desired speed in y-dir
  void move(double x_vec, double rot_vec)
  {
    auto message = geometry_msgs::msg::Twist();
    message.angular.z = rot_vec;
    message.linear.x = x_vec;
    publisher_->publish(message);
  }

  /// @brief Moves the robot in a predetermined path then stops
  void timer_callback()
  {
    counter_++;
    if (counter_ <= 1300) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 1300 && counter_ <= 1700) {
      move(0.0, 0.3927); // Twist Left
    } else if (counter_ > 1700 && counter_ <= 2200) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 2200 && counter_ <= 2600) {
      move(0.0, 0.3927); // Twist Left
    } else if (counter_ > 2600 && counter_ <= 4200) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 4200 && counter_ <= 4600) {
      move(0.0, -0.3927);  // Twist Right
    } else if (counter_ > 4600 && counter_ <= 5100) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 5100 && counter_ <= 5500) {
      move(0.0, -0.3927);  // Twist Right
    } else if (counter_ > 5500 && counter_ <= 7100) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 7100 && counter_ <= 7500) {
      move(0.0, 0.3927);  // Twist Left
    } else if (counter_ > 7500 && counter_ <= 8000) {
      move(0.2, 0.0); // Move forward
    } else if (counter_ > 8000 && counter_ <= 8400) {
      move(0.0, 0.3927);  // Twist Left
    } else if (counter_ > 8400 && counter_ <= 10000) {
      move(0.2, 0.0); // Move forward
      // } else if (counter_ > 8000) {
      //   counter_ = 0;  // Reset the counter
    } else {
      move(0.0, 0.0);  // Stay Still
    }

    // Print error when complete
    if (counter_ == 10000) {
      publish_error();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Choreographed>());
  rclcpp::shutdown();
  return 0;
}
