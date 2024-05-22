#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>

class SimpleMoveNode : public rclcpp::Node
{
public:
  SimpleMoveNode()
  : Node("simple_move_node")
  {
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&SimpleMoveNode::laser_scan_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize default values
    forward_speed_ = 0.2;      // meters per second
    turn_speed_ = 0.5;         // radians per second
    clear_threshold_ = 0.5;    // meters
  }

private:
  // Objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Variables
  double forward_speed_;
  double turn_speed_;
  double clear_threshold_;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int ranges_size = msg->ranges.size();
    if (ranges_size == 0) {
      RCLCPP_WARN(this->get_logger(), "No laser scan data received");
      return;
    }

    // Check the front area
    bool is_clear_ahead = std::all_of(
      msg->ranges.begin() + ranges_size / 3, msg->ranges.begin() + 2 * ranges_size / 3,
      [this](float range) {return range > clear_threshold_;});

    if (is_clear_ahead) {
      // Move forward
      move_forward();
    } else {
      // Check left and right
      bool is_clear_left = std::all_of(
        msg->ranges.begin(), msg->ranges.begin() + ranges_size / 3,
        [this](float range) {return range > clear_threshold_;});
      bool is_clear_right = std::all_of(
        msg->ranges.begin() + 2 * ranges_size / 3, msg->ranges.end(),
        [this](float range) {return range > clear_threshold_;});

      if (is_clear_left && is_clear_right) {
        // Turn randomly if both sides are clear
        if (rand() % 2 == 0) {
          turn_left();
        } else {
          turn_right();
        }
      } else if (is_clear_left) {
        turn_left();
      } else if (is_clear_right) {
        turn_right();
      } else {
        // If no clear path, stop
        stop();
      }
    }
  }

  void move_forward()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forward_speed_;
    msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(msg);
  }

  void turn_left()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = turn_speed_;
    cmd_vel_publisher_->publish(msg);
  }

  void turn_right()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = -turn_speed_;
    cmd_vel_publisher_->publish(msg);
  }

  void stop()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMoveNode>());
  rclcpp::shutdown();
  return 0;
}
