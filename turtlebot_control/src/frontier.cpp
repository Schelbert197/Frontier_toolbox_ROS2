#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

class FrontierExplorationNode : public rclcpp::Node
{
public:
    FrontierExplorationNode()
        : Node("frontier_exploration_node")
    {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&FrontierExplorationNode::map_callback, this, std::placeholders::_1));

        // Initialize other variables and publishers here
    }

private:

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    nav_msgs::msg::OccupancyGrid current_map_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        find_frontiers();
    }

    void find_frontiers()
    {
        std::vector<geometry_msgs::msg::Point> frontiers;

        // Loop through each cell in the occupancy grid
        for (int y = 0; y < current_map_.info.height; ++y)
        {
            for (int x = 0; x < current_map_.info.width; ++x)
            {
                int index = x + y * current_map_.info.width;
                if (current_map_.data[index] == 0) // Free space
                {
                    if (is_frontier_cell(x, y))
                    {
                        geometry_msgs::msg::Point frontier;
                        frontier.x = x * current_map_.info.resolution + current_map_.info.origin.position.x;
                        frontier.y = y * current_map_.info.resolution + current_map_.info.origin.position.y;
                        frontiers.push_back(frontier);
                    }
                }
            }
        }

        // Perform clustering and select target frontier here
        if (!frontiers.empty())
        {
            cluster_frontiers(frontiers);
            geometry_msgs::msg::Point target_frontier = select_target_frontier(frontiers);

            // Plan and move to the target frontier
            plan_and_move_to_frontier(target_frontier);
        }
    }

    bool is_frontier_cell(int x, int y)
    {
        // Check the 4-connected neighbors to see if any of them are unknown
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0) continue;

                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < current_map_.info.width && ny >= 0 && ny < current_map_.info.height)
                {
                    int neighbor_index = nx + ny * current_map_.info.width;
                    if (current_map_.data[neighbor_index] == -1) // Unknown space
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void cluster_frontiers(const std::vector<geometry_msgs::msg::Point> &frontiers)
    {
        // Implement clustering logic (e.g., DBSCAN or k-means clustering)
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorationNode>());
    rclcpp::shutdown();
    return 0;
}
