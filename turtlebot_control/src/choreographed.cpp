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
      std::chrono::milliseconds(5),  // 200Hz -> 1000ms / 200 = 5ms
      std::bind(&Choreographed::move, this)
    );
  }

private:
  void move()
  {
    counter_++;
    if (counter_ >= 100 && counter_ <= 200) {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.5;  // Move forward
      publisher_->publish(message);
    } else if (counter_ > 200) {
      counter_ = 0;  // Reset the counter
    } else {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.0;  // Stay Still
      publisher_->publish(message);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Choreographed>());
  rclcpp::shutdown();
  return 0;
}
