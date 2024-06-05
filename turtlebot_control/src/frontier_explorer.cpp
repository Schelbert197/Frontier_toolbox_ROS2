#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <queue>
#include <set>
#include <cmath>

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/exploration_goal",
      10);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    auto frontiers = findFrontiers(msg);
    if (!frontiers.empty()) {
      geometry_msgs::msg::Point robot_position;
      if (getRobotPosition(robot_position)) {
        auto closest_goal = getClosestGoal(frontiers, robot_position);
        publishGoal(closest_goal);
      }
    }
  }

  bool getRobotPosition(geometry_msgs::msg::Point & robot_position)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform 'base_link' to 'map': %s", ex.what());
      return false;
    }

    robot_position.x = transform_stamped.transform.translation.x;
    robot_position.y = transform_stamped.transform.translation.y;
    robot_position.z = transform_stamped.transform.translation.z;
    return true;
  }

  geometry_msgs::msg::Point getClosestGoal(
    const std::vector<geometry_msgs::msg::Point> & frontiers,
    const geometry_msgs::msg::Point & robot_position)
  {
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point closest_goal;

    for (const auto & frontier : frontiers) {
      double distance = std::hypot(frontier.x - robot_position.x, frontier.y - robot_position.y);
      if (distance < min_distance) {
        min_distance = distance;
        closest_goal = frontier;
      }
    }

    return closest_goal;
  }

  std::vector<geometry_msgs::msg::Point> findFrontiers(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    std::vector<geometry_msgs::msg::Point> frontiers;
    std::vector<bool> visited(map->data.size(), false);

    for (unsigned int y = 0; y < map->info.height; ++y) {
      for (unsigned int x = 0; x < map->info.width; ++x) {
        unsigned int index = x + y * map->info.width;
        if (map->data[index] == -1 && !visited[index]) {
          if (isFrontierCell(x, y, map)) {
            auto frontier = growFrontier(x, y, map, visited);
            if (!frontier.empty()) {
              frontiers.push_back(frontier.front());
            }
          }
        }
      }
    }
    return frontiers;
  }

  bool isFrontierCell(
    unsigned int x, unsigned int y,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    int width = map->info.width;
    int height = map->info.height;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        int nx = x + dx;
        int ny = y + dy;

        if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
          unsigned int index = nx + ny * width;
          if (map->data[index] == 0) {
            return true;
          }
        }
      }
    }
    return false;
  }

  std::vector<geometry_msgs::msg::Point> growFrontier(
    unsigned int start_x, unsigned int start_y,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map, std::vector<bool> & visited)
  {
    std::vector<geometry_msgs::msg::Point> frontier;
    std::queue<std::pair<unsigned int, unsigned int>> bfs_queue;
    bfs_queue.push({start_x, start_y});

    int width = map->info.width;
    int height = map->info.height;

    while (!bfs_queue.empty()) {
      auto [x, y] = bfs_queue.front();
      bfs_queue.pop();

      unsigned int index = x + y * width;
      if (visited[index]) {
        continue;
      }
      visited[index] = true;

      if (map->data[index] == -1 && isFrontierCell(x, y, map)) {
        geometry_msgs::msg::Point p;
        p.x = x * map->info.resolution + map->info.origin.position.x;
        p.y = y * map->info.resolution + map->info.origin.position.y;
        p.z = 0.0;
        frontier.push_back(p);

        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
              unsigned int neighbor_index = nx + ny * width;
              if (!visited[neighbor_index] && map->data[neighbor_index] == -1) {
                bfs_queue.push({nx, ny});
              }
            }
          }
        }
      }
    }

    return frontier;
  }

  void publishGoal(const geometry_msgs::msg::Point & goal)
  {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position = goal;
    goal_msg.pose.orientation.w = 1.0;
    goal_publisher_->publish(goal_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <vector>

// class FrontierExplorationNode : public rclcpp::Node
// {
// public:
//     FrontierExplorationNode()
//         : Node("frontier_exploration_node")
//     {
//         map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//             "/map", 10, std::bind(&FrontierExplorationNode::map_callback, this, std::placeholders::_1));

//         // Initialize other variables and publishers here
//     }

// private:

//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
//     nav_msgs::msg::OccupancyGrid current_map_;

//     void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//     {
//         current_map_ = *msg;
//         find_frontiers();
//     }

//     void find_frontiers()
//     {
//         std::vector<geometry_msgs::msg::Point> frontiers;

//         // Loop through each cell in the occupancy grid
//         for (int y = 0; y < current_map_.info.height; ++y)
//         {
//             for (int x = 0; x < current_map_.info.width; ++x)
//             {
//                 int index = x + y * current_map_.info.width;
//                 if (current_map_.data[index] == 0) // Free space
//                 {
//                     if (is_frontier_cell(x, y))
//                     {
//                         geometry_msgs::msg::Point frontier;
//                         frontier.x = x * current_map_.info.resolution + current_map_.info.origin.position.x;
//                         frontier.y = y * current_map_.info.resolution + current_map_.info.origin.position.y;
//                         frontiers.push_back(frontier);
//                     }
//                 }
//             }
//         }

//         // Perform clustering and select target frontier here
//         if (!frontiers.empty())
//         {
//             cluster_frontiers(frontiers);
//             geometry_msgs::msg::Point target_frontier = select_target_frontier(frontiers);

//             // Plan and move to the target frontier
//             plan_and_move_to_frontier(target_frontier);
//         }
//     }

//     bool is_frontier_cell(int x, int y)
//     {
//         // Check the 4-connected neighbors to see if any of them are unknown
//         for (int dx = -1; dx <= 1; ++dx)
//         {
//             for (int dy = -1; dy <= 1; ++dy)
//             {
//                 if (dx == 0 && dy == 0) continue;

//                 int nx = x + dx;
//                 int ny = y + dy;
//                 if (nx >= 0 && nx < current_map_.info.width && ny >= 0 && ny < current_map_.info.height)
//                 {
//                     int neighbor_index = nx + ny * current_map_.info.width;
//                     if (current_map_.data[neighbor_index] == -1) // Unknown space
//                     {
//                         return true;
//                     }
//                 }
//             }
//         }
//         return false;
//     }

//     void cluster_frontiers(const std::vector<geometry_msgs::msg::Point> &frontiers)
//     {
//         // Implement clustering logic (e.g., DBSCAN or k-means clustering)
//     }

// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FrontierExplorationNode>());
//     rclcpp::shutdown();
//     return 0;
// }
