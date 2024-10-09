#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

class PointCloudToLaserScan : public rclcpp::Node
{
public:
    PointCloudToLaserScan(const rclcpp::NodeOptions & options)
  : Node("pointcloud_to_laserscan", options)
  {
    // Declare and get parameters
    scan_height_ = this->declare_parameter("scan_height", 0.1);
    range_min_ = this->declare_parameter("range_min", 0.0);
    range_max_ = this->declare_parameter("range_max", 100.0);
    scan_frame_ = this->declare_parameter<std::string>("scan_frame", "base_link");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");

    // Subscription and publisher
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensors/lidar_3d/velodyne_points", 10,
      std::bind(&PointCloudToLaserScan::pointcloud_callback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
  }

private:
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  // Parameters
  double scan_height_;
  double range_min_;
  double range_max_;
  std::string scan_frame_;
  std::string scan_topic_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert the PointCloud2 message to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Filter the cloud based on height (z-axis) to reduce noise outside the scan height
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-scan_height_ / 2.0, scan_height_ / 2.0);
    pass.filter(*cloud);

    // Prepare the LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = scan_frame_;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0;  // 1-degree increment
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    int num_readings = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    std::vector<float> ranges(num_readings, std::numeric_limits<float>::infinity());

    // Process the filtered point cloud
    for (const auto& point : cloud->points) {
      float x = point.x;
      float y = point.y;
      float distance = std::sqrt(x * x + y * y);

      // Calculate angle and set range if within min/max limits
      if (distance >= range_min_ && distance <= range_max_) {
        float angle = std::atan2(y, x);
        int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);

        if (index >= 0 && index < num_readings) {
          ranges[index] = std::min(ranges[index], distance);
        }
      }
    }

    scan.ranges = ranges;
    scan_pub_->publish(scan);
  }


};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudToLaserScan)
