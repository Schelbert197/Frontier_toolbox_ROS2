#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>

using std::placeholders::_1;

class MapComparator : public rclcpp::Node
{
public:
  MapComparator()
  : Node("map_comparator")
  {
    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/ground_truth_map", 10, std::bind(&MapComparator::groundTruthCallback, this, _1));
    test_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/test_map", 10, std::bind(&MapComparator::testMapCallback, this, _1));
  }

private:
  void groundTruthCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    ground_truth_map_ = msg;
    compareMaps();
  }

  void testMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    test_map_ = msg;
    compareMaps();
  }

  void compareMaps()
  {
    if (!ground_truth_map_ || !test_map_) {
      return;
    }

    // Ensure the maps are the same size
    if (ground_truth_map_->info.width != test_map_->info.width ||
      ground_truth_map_->info.height != test_map_->info.height)
    {
      RCLCPP_ERROR(this->get_logger(), "Maps must be the same size");
      return;
    }

    int matches = 0;
    int total_cells = ground_truth_map_->info.width * ground_truth_map_->info.height;

    for (int i = 0; i < total_cells; ++i) {
      int gt_value = ground_truth_map_->data[i];
      int tst_value = test_map_->data[i];

      // Convert to binary form (0 for free, 1 for occupied, -1 for unknown)
      int gt_binary = (gt_value == 100) ? 1 : (gt_value == 0) ? 0 : -1;
      int tst_binary = (tst_value == 100) ? 1 : (tst_value == 0) ? 0 : -1;

      // Consider unknown cells as mismatches
      if (gt_binary != -1 && tst_binary != -1) {
        if (gt_binary == tst_binary) {
          matches++;
        }
      }
    }

    double similarity_percentage = (static_cast<double>(matches) / total_cells) * 100.0;
    RCLCPP_INFO(this->get_logger(), "Map Similarity: %.2f%%", similarity_percentage);

    // Reset the maps after comparison
    ground_truth_map_.reset();
    test_map_.reset();
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr test_map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr ground_truth_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr test_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapComparator>());
  rclcpp::shutdown();
  return 0;
}
