#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Choreographed : public rclcpp::Node
{
public:
  Choreographed()
  : Node("choreographed"), counter_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
  rclcpp::TimerBase::SharedPtr timer_;
  // Variables
  int counter_ = -100;
  double current_error_ = 0.0;

  void error_callback(const std_msgs::msg::Float32 err)
  {
    current_error_ = err.data;
  }

  /// @brief Creates the Twist msg object and publishes it
  /// @param x_vec
  /// @param rot_vec
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
      RCLCPP_INFO(this->get_logger(), "Total Aggregated and Normalized error: %f", current_error_);
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
