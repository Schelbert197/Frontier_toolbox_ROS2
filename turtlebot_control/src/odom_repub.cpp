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