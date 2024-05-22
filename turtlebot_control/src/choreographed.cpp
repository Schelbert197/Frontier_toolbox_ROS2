#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Choreographed : public rclcpp::Node
{
public:
  Choreographed()
  : Node("choreographed"), counter_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 200Hz -> 1000ms / 100 = 10ms
      std::bind(&Choreographed::move, this)
    );
  }

private:
  // Objects
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Variables
  int counter_;

  void move()
  {
    counter_++;
    auto message = geometry_msgs::msg::Twist();
    if (counter_ <= 1000) {
      message.linear.x = 0.25;  // Move forward
      publisher_->publish(message);
    } else if (counter_ > 1000 && counter_ <= 1200) {
      message.linear.x = 0.0;  // Twist Left
      message.angular.z = 0.78539;
      publisher_->publish(message);
    } else if (counter_ > 1300 && counter_ <= 1600) {
      message.linear.x = 0.25;  // Move forward
      publisher_->publish(message);
    } else if (counter_ > 1600 && counter_ <= 1800) {
      message.linear.x = 0.0;  // Twist Left
      message.angular.z = 0.78539;
      publisher_->publish(message);
    } else if (counter_ > 1700 && counter_ <= 2300) {
      message.linear.x = 0.25;  // Move forward
      publisher_->publish(message);
    } else if (counter_ > 2300 && counter_ <= 2500) {
      message.linear.x = 0.0;  // Twist Right
      message.angular.z = -0.78539;
      publisher_->publish(message);
    } else if (counter_ > 2500 && counter_ <= 3000) {
      message.linear.x = 0.25;  // Move forward
      publisher_->publish(message);
    } else if (counter_ > 8000) {
      counter_ = 0;  // Reset the counter
    } else {
      message.linear.x = 0.0;  // Stay Still
      publisher_->publish(message);
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
