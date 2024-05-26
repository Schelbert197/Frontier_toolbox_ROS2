#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
  : Node("path_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PathPublisher::odom_callback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot_path", 10);
    map_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot_map_path", 10);


    path_.header.frame_id = "map";  // Change this to "map" if you want the path in the map frame
    map_path_.header.frame_id = "map"; // "map" frame to see skewed path
  }

private:
// Create Objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_path_pub_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path map_path_;

  // Tf objects
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Create pose object for both things
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    // Create and publish path to odom frame
    path_.poses.push_back(pose_stamped);
    path_.header.stamp = this->now();
    path_pub_->publish(path_);

    // Create and publish path to map frame
    try {
      // Transform the pose to the map frame
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration(1, 0));

      geometry_msgs::msg::PoseStamped map_pose_stamped;
      tf2::doTransform(pose_stamped, map_pose_stamped, transform_stamped);

      map_path_.poses.push_back(map_pose_stamped);
      map_path_.header.stamp = this->now();

      map_path_pub_->publish(map_path_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not transform %s to map: %s",
        msg->header.frame_id.c_str(), ex.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
