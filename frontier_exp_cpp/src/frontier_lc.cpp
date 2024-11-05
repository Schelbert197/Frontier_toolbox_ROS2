#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_client_cpp/srv/nav_to_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include <lifecycle_msgs/srv/get_state.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <random>
#include <algorithm>


class FrontierExplorationNode : public nav2_util::LifecycleNode,
  public std::enable_shared_from_this<FrontierExplorationNode>
{
public:
  FrontierExplorationNode()
  : nav2_util::LifecycleNode("frontier_explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "Frontier explorer lifecycle node initialized");

    // Init Descriptions
    auto use_naive_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_action_client_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto viewpoint_depth_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto robot_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_calc_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_sampling_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto sampling_threshold_des = rcl_interfaces::msg::ParameterDescriptor{};

    // Parameter Descriptions and Declarations
    use_naive_des.description = "Choose to use naive frontier selection or entropy reduction";
    declare_parameter("use_naive", false, use_naive_des);
    use_action_client_des.description =
      "Choose to use custom action client node (true) or just publish goal pose to nav2 (false)";
    declare_parameter("use_action_client_node", false, use_action_client_des);
    viewpoint_depth_des.description =
      "The distance in front of the robot that the naive algorithm chooses a frontier by min dist [m]";
    declare_parameter("viewpoint_depth", 1.0, viewpoint_depth_des);
    entropy_radius_des.description =
      "The radius around a frontier considered in a state update estimation";
    declare_parameter("entropy_radius", 2.5, entropy_radius_des);
    robot_radius_des.description =
      "The radius around the robot in [m] to determine if a frontier is too close";
    declare_parameter("robot_radius", 0.35, robot_radius_des);
    entropy_calc_des.description =
      "Choose whether to calculate entropy or select based on raw unknown count";
    declare_parameter("use_entropy_calc", false, entropy_calc_des);
    use_sampling_des.description =
      "Choose whether to use sampling to decrease computational load with many frontiers";
    declare_parameter("use_sampling", false, use_sampling_des);
    sampling_threshold_des.description =
      "The threshold at which after this many frontiers, the calculation will be sampled";
    declare_parameter("sampling_threshold", 500, sampling_threshold_des);

    // Get Parameters
    is_sim_ = get_parameter("use_sim_time").as_bool();
    use_naive_ = get_parameter("use_naive").as_bool();
    use_action_client_ = get_parameter("use_action_client_node").as_bool();
    viewpoint_depth_ = get_parameter("viewpoint_depth").as_double();
    entropy_radius_ = get_parameter("entropy_radius").as_double();
    robot_radius_ = get_parameter("robot_radius").as_double();
    use_entropy_calc_ = get_parameter("use_entropy_calc").as_bool();
    use_sampling_ = get_parameter("use_sampling").as_bool();
    sampling_threshold = get_parameter("sampling_threshold").as_int();

    // Create separate callback groups for each service client
    nav_to_pose_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    cancel_nav_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize service clients as null, to be created in on_configure()
    nav_to_pose_client_ = nullptr;
    cancel_nav_client_ = nullptr;

    // Trigger configuration during node startup
    auto current_state = this->get_current_state();
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      this->configure();
    }
  }

  // Lifecycle transition callbacks
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring...");
    // Subscribe to the /map topic
    map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&FrontierExplorationNode::mapCallback, this, std::placeholders::_1));

    // Publisher for goal pose
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    map_with_frontiers_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map_with_frontiers",
      10);

    // Subscriber for goal success
    goal_sucess_subscriber_ = create_subscription<std_msgs::msg::String>(
      "jackal_goal", 10, std::bind(
        &FrontierExplorationNode::goalPubCallback, this,
        std::placeholders::_1));
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    goal_publisher_->on_activate();
    map_with_frontiers_pub_->on_activate();

    // Create the service clients in on_activate
    nav_to_pose_client_ = this->create_client<nav_client_cpp::srv::NavToPose>(
      "jackal_nav_to_pose", rmw_qos_profile_services_default, nav_to_pose_callback_group_);

    cancel_nav_client_ = this->create_client<std_srvs::srv::Empty>(
      "jackal_cancel_nav", rmw_qos_profile_services_default, cancel_nav_callback_group_);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    goal_publisher_->on_deactivate();
    map_with_frontiers_pub_->on_deactivate();
    // Reset navigation clients
    nav_to_pose_client_.reset();
    cancel_nav_client_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    map_subscriber_.reset();
    // Reset navigation clients for secure cleanup
    nav_to_pose_client_.reset();
    cancel_nav_client_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  // Subs and pubs
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
  goal_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>>
  map_with_frontiers_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sucess_subscriber_;

  // Services for action node
  rclcpp::CallbackGroup::SharedPtr nav_to_pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_nav_callback_group_;

  rclcpp::Client<nav_client_cpp::srv::NavToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cancel_nav_client_;

  // Data objects
  nav_msgs::msg::OccupancyGrid map_data_;
  std::vector<std::pair<int, int>> frontiers_;
  std::vector<std::pair<int, int>> sampled_frontiers_;
  std::vector<double> entropies_;
  std::vector<int> unknowns_;
  std::pair<double, double> robot_position_;

  double viewpoint_depth_;
  bool is_sim_;
  double entropy_radius_ = 2.5;
  bool use_naive_;
  bool use_entropy_calc_;
  bool use_sampling_;
  int sampling_threshold;
  bool use_action_client_;
  int best_frontier_idx_;
  double robot_radius_;

  // Buffer objects
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  void activation_callback()
  {
    // stuff
    auto current_state = this->get_current_state();

    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(get_logger(), "Callback is present. Triggering activation.");
      this->activate();
    }
  }

  void goalPubCallback(const std_msgs::msg::String::SharedPtr msg)
  {

    if (msg->data == "Succeeded") {
      RCLCPP_INFO(get_logger(), "Success, waiting for arrival.");
    } else if (msg->data == "Aborted") {
      RCLCPP_WARN(get_logger(), "Goal aborted... selecting next candidate.");
      if (use_entropy_calc_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
        selectNextBest(sampled_frontiers_);
      } else {
        selectNextBest(frontiers_);
      }
    } else if (msg->data == "Canceled") {
      RCLCPP_WARN(get_logger(), "Goal Cancelled.");
    } else if (msg->data == "Rejected") {
      RCLCPP_WARN(get_logger(), "Goal rejected... selecting next candidate.");
      if (use_entropy_calc_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
        selectNextBest(sampled_frontiers_);
      } else {
        selectNextBest(frontiers_);
      }
    } else if (msg->data == "Unknown") {
      RCLCPP_DEBUG(get_logger(), "Unknown Response");
    } else {
      RCLCPP_WARN(get_logger(), "Rogue Unknown Response");
    }
  }

  void selectNextBest(std::vector<std::pair<int, int>> & frontiers)
  {
    if (best_frontier_idx_ >= 0 && best_frontier_idx_ < static_cast<int>(entropies_.size())) {
      RCLCPP_INFO(get_logger(), "Replanning and selecting next best goal...");
      if (use_entropy_calc_) {
        // Erase most recently selected frontier and its entropy
        entropies_.erase(entropies_.begin() + best_frontier_idx_);
        frontiers.erase(frontiers.begin() + best_frontier_idx_);

        // Find and return next best entropy and index
        auto [index, entropy] = bestEntropyIndexScore();
        best_frontier_idx_ = index;
        RCLCPP_INFO(
          get_logger(), "Selecting frontier %d, with entropy reduction %f", best_frontier_idx_,
          entropy);
      } else {
        // Erase most recently selected frontier and its score
        unknowns_.erase(unknowns_.begin() + best_frontier_idx_);
        frontiers.erase(frontiers.begin() + best_frontier_idx_);

        // Find and return next best score and index
        auto [index, score] = bestUnknownsIndexScore();
        best_frontier_idx_ = index;
        RCLCPP_INFO(
          get_logger(), "Selecting frontier %d, with best score %d", best_frontier_idx_,
          score);
      }

      publishToNav2ActionClient(frontiers.at(best_frontier_idx_));
    } else {
      RCLCPP_WARN(get_logger(), "Mismatch in index. Skipping reset...");
    }
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_data_ = *msg;

    auto current_state = this->get_current_state();
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      // Get the robot's current viewpoint position from the transform
      auto robot_vp_position = getRobotViewpoint();
      if (robot_vp_position) {
        findFrontiers();
        cleanupFrontiers();
        publishGoalFrontier();
        publishMapWithFrontiers();
      } else {
        RCLCPP_WARN(get_logger(), "Unable to determine robot's position.");
      }
    } else {
      RCLCPP_WARN(get_logger(), "Node inactive... awaiting activation");
    }
  }

  std::optional<std::pair<double, double>> getRobotViewpoint()
  {
    std::string base_frame = is_sim_ ? "base_footprint" : "base_link";
    try {
      auto transform = tf_buffer_.lookupTransform("map", base_frame, tf2::TimePointZero);

      // Extract position and orientation from the transform
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);

      robot_position_ = std::make_pair(x, y);

      // Use setRPY to get yaw
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      if (use_naive_ == false) {
        // Make VP 0 to allow DTR calculations later
        viewpoint_depth_ = 0.0;
      }
      // Adjust x and y based on the viewpoint depth
      double x_adj = x + std::cos(yaw) * viewpoint_depth_;
      double y_adj = y + std::sin(yaw) * viewpoint_depth_;

      RCLCPP_INFO(get_logger(), "Got new robot position at x,y: %f, %f", x, y);
      return std::make_optional(std::make_pair(x_adj, y_adj));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Error getting transform: %s", ex.what());
      return std::nullopt;
    }
  }

  void findFrontiers()
  {
    // Loop through the map and find frontiers
    int height = map_data_.info.height;
    int width = map_data_.info.width;
    const auto & data = map_data_.data;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        if (data[idx] == -1 && hasFreeNeighbor(x, y)) {
          frontiers_.emplace_back(x, y);
        }
      }
    }
    // Convert the vector to a string
    std::stringstream ss;
    ss << "Vector contents: ";
    for (const auto & pair : frontiers_) {
      ss << "(" << pair.first << ", " << pair.second << ") ";
    }
    // Print the vector contents of the frontier vector
    RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
  }

  bool hasFreeNeighbor(int x, int y)
  {
    int height = map_data_.info.height;
    int width = map_data_.info.width;
    const auto & data = map_data_.data;

    std::vector<std::pair<int, int>> neighbors = {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
    for (const auto & [nx, ny] : neighbors) {
      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        int idx = ny * width + nx;
        if (data[idx] == 0) {       // Free cell
          return true;
        }
      }
    }
    return false;
  }

  void cleanupFrontiers()
  {
    std::vector<std::pair<int, int>> valid_frontiers;
    const auto & data = map_data_.data;

    for (const auto & frontier : frontiers_) {
      int idx = frontier.second * map_data_.info.width + frontier.first;
      if (data[idx] == -1 &&
        hasFreeNeighbor(frontier.first, frontier.second) && !tooClose(frontier))
      {
        valid_frontiers.push_back(frontier);
      }
    }
    frontiers_ = valid_frontiers;
  }

  bool tooClose(const std::pair<int, int> & frontier)
  {
    return distanceToRobot(frontier) <= robot_radius_;
  }

  void publishGoalFrontier()
  {
    if (frontiers_.empty()) {
      RCLCPP_INFO(get_logger(), "No frontiers available.");
      return;
    }

    std::pair<int, int> goal_frontier;
    if (use_naive_ == true) {
      goal_frontier = *std::min_element(
        frontiers_.begin(), frontiers_.end(), [this](const auto & f1, const auto & f2)
        {
          return distanceToRobot(f1) < distanceToRobot(f2);
        });
    } else if (use_sampling_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
      goal_frontier = bestScoringFrontier(sampleRandomFrontiers(frontiers_, sampling_threshold));
      RCLCPP_DEBUG(get_logger(), "Using sampling to get best frontier");
    } else {
      goal_frontier = bestScoringFrontier(frontiers_);
    }

    if (!use_action_client_) {
      publishToNav2PlannerServer(goal_frontier);
    } else {
      publishToNav2ActionClient(goal_frontier);
    }
  }

  void publishToNav2ActionClient(const std::pair<int, int> & goal_frontier)
  {
    auto goal_pose = std::make_shared<nav_client_cpp::srv::NavToPose::Request>();
    std::tie(goal_pose->x, goal_pose->y) = cellToWorld(goal_frontier);
    goal_pose->theta = 0.0;
    RCLCPP_INFO(
      get_logger(), "Publishing goal at %f, %f", goal_pose->x, goal_pose->y);
    auto future_result = nav_to_pose_client_->async_send_request(goal_pose);
  }

  void publishToNav2PlannerServer(const std::pair<int, int> & goal_frontier)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    std::tie(goal_pose.pose.position.x, goal_pose.pose.position.y) = cellToWorld(goal_frontier);
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    goal_publisher_->publish(goal_pose);
    RCLCPP_INFO(
      get_logger(), "Publishing goal at %f, %f", goal_pose.pose.position.x,
      goal_pose.pose.position.y);
  }

  void publishMapWithFrontiers()
  {
    auto modified_map = map_data_;

    for (const auto & frontier : frontiers_) {
      int idx = frontier.second * modified_map.info.width + frontier.first;
      modified_map.data[idx] = 50;       // Mark frontiers in the map
    }

    map_with_frontiers_pub_->publish(modified_map);
    RCLCPP_INFO(get_logger(), "Published map with highlighted frontiers.");
  }

  double distanceToRobot(const std::pair<int, int> & frontier)
  {
    auto [fx, fy] = cellToWorld(frontier);
    auto [rx, ry] = robot_position_;
    return std::hypot(fx - rx, fy - ry);
  }

  std::pair<double, double> cellToWorld(const std::pair<int, int> & cell)
  {
    double world_x = map_data_.info.origin.position.x + (cell.first * map_data_.info.resolution);
    double world_y = map_data_.info.origin.position.y + (cell.second * map_data_.info.resolution);
    return {world_x, world_y};
  }

  int countUnknownCellsWithinRadius(int index, double rad)
  {
    int unknown_count = 0;

    // Get map metadata
    int width = static_cast<int>(map_data_.info.width);
    int height = static_cast<int>(map_data_.info.height);
    double resolution = map_data_.info.resolution;

    // Calculate the center cell's row and column from the index
    int center_row = index / width;
    int center_col = index % width;

    // Determine the search range in cells based on the radius
    int range = static_cast<int>(std::round(rad / resolution));

    // Loop through the square neighborhood around the center cell
    for (int row = center_row - range; row <= center_row + range; ++row) {
      for (int col = center_col - range; col <= center_col + range; ++col) {
        // Skip cells outside the grid boundaries
        if (row < 0 || row >= height || col < 0 || col >= width) {
          continue;
        }

        // Compute the Euclidean distance from the center cell
        double dist =
          std::sqrt(std::pow(row - center_row, 2) + std::pow(col - center_col, 2)) * resolution;

        // Only consider cells within the specified radius
        if (dist <= rad) {
          // Calculate the index of the current cell in the OccupancyGrid data
          int cell_index = row * width + col;

          // Check if the cell is unknown (-1)
          if (map_data_.data[cell_index] == -1 &&
            !occluded(col, row, center_col, center_row, width, map_data_.data))
          {
            unknown_count++;
          }
        }
      }
    }

    return unknown_count;
  }

  std::vector<std::pair<int, int>> sampleRandomFrontiers(
    const std::vector<std::pair<int,
    int>> & frontiers, size_t sample_size)
  {
    sampled_frontiers_.clear();

    // No need to sample if frontiers is already small enough
    if (frontiers.size() <= sample_size) {
      return frontiers;
    }

    // Generate a list of indices from 0 to frontiers.size() - 1
    std::vector<size_t> indices(frontiers.size());
    std::iota(indices.begin(), indices.end(), 0);   // Fill with 0, 1, ..., frontiers.size()-1

    // Set up random engine and shuffle the indices
    std::random_device rd;
    std::mt19937 gen(rd());    // Mersenne Twister engine
    std::shuffle(indices.begin(), indices.end(), gen);

    // Select the first sample_size indices
    for (size_t i = 0; i < sample_size; ++i) {
      sampled_frontiers_.push_back(frontiers[indices[i]]);
    }

    return sampled_frontiers_;
  }

  std::pair<int, int> bestScoringFrontier(const std::vector<std::pair<int, int>> & frontiers)
  {
    double total_entropy;
    // Establish baseline and reset for preferred scoring approach
    if (use_entropy_calc_) {
      total_entropy = calculateMapEntropy();
      RCLCPP_INFO(get_logger(), "Total Map Entropy %f", total_entropy);
      // Reset list of entropies
      entropies_.clear();
    } else {
      // reset unknowns list
      unknowns_.clear();
    }

    // Loop through all frontiers and get score
    for (size_t i = 0; i < frontiers.size(); ++i) {
      const auto & frontier = frontiers.at(i);
      int idx = frontier.second * map_data_.info.width + frontier.first;
      int unknowns = countUnknownCellsWithinRadius(idx, entropy_radius_);

      if (use_entropy_calc_) {
        // calculate current reduced entropy and place in list
        entropies_.emplace_back(
          total_entropy - (unknowns * calculateEntropy(-1)) +
          (unknowns * calculateEntropy(0)));
      } else {
        unknowns_.emplace_back(unknowns);
      }
    }

    if (use_entropy_calc_) {
      // Find and return best entropy and index
      auto [index, entropy] = bestEntropyIndexScore();
      best_frontier_idx_ = index;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with entropy reduction %f", best_frontier_idx_,
        entropy);
    } else {
      // Find and return best score and index
      auto [index, score] = bestUnknownsIndexScore();
      best_frontier_idx_ = index;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with best score %d", best_frontier_idx_,
        score);
    }

    return frontiers.at(best_frontier_idx_);
  }

  std::pair<int, double> bestEntropyIndexScore()
  {
    // Select least entropy from list and find index
    auto min_iterator = std::min_element(entropies_.begin(), entropies_.end());
    double best_possible_entropy = *min_iterator;
    int best_frontier_idx_ = std::distance(entropies_.begin(), min_iterator);

    return std::make_pair(best_frontier_idx_, best_possible_entropy);
  }

  std::pair<int, int> bestUnknownsIndexScore()
  {
    // Select most converted unknowns from list and find index
    auto max_iterator = std::max_element(unknowns_.begin(), unknowns_.end());
    int best_possible_unknowns = *max_iterator;
    int best_frontier_idx_ = std::distance(unknowns_.begin(), max_iterator);

    return std::make_pair(best_frontier_idx_, best_possible_unknowns);
  }

  double calculateMapEntropy()
  {
    double entropy;
    for (const auto & cell : map_data_.data) {
      entropy += calculateEntropy(cell);
    }
    return entropy;
  }

  double calculateEntropy(int cell_value)
  {
    double v;
    if (cell_value == -1) {
      v = 0.5;
    } else if (cell_value == 0) {
      v = 0.01;
    } else if (cell_value == 100) {
      v = 0.99;
    }
    return -1 * ((v * log(v)) + ((1 - v) * log(1 - v)));
  }

  bool occluded(int x1, int y1, int x2, int y2, int width, const std::vector<int8_t> & map_data)
  {
    // Bresenham's line algorithm to generate points between (x1, y1) and (x2, y2)
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (x1 != x2 || y1 != y2) {
      // Check if the current cell is occupied (value 100 means occupied)
      int cell_index = y1 * width + x1;
      if (map_data[cell_index] == 100) {
        return true;  // There is an occupied cell between the two points
      }

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x1 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y1 += sy;
      }
    }

    return false;  // No occupied cells found between the two points
  }

};

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(FrontierExplorationNode)
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<FrontierExplorationNode> lc_node =
    std::make_shared<FrontierExplorationNode>();
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();

  return 0;
}
