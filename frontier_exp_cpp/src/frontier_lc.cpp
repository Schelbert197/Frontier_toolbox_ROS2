#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_client_cpp/srv/nav_to_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include <lifecycle_msgs/srv/get_state.hpp>
#include "frontier_exp_cpp/frontier_helper.hpp"
#include <opencv2/core.hpp>
#include <map>
#include <optional>
#include <cstdlib> // For generating random colors


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
    auto use_clustering_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_action_client_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto viewpoint_depth_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto robot_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_calc_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_sampling_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto sampling_threshold_des = rcl_interfaces::msg::ParameterDescriptor{};

    // Parameter Descriptions and Declarations
    use_naive_des.description = "Choose to use naive frontier selection or mutual information";
    declare_parameter("use_naive", false, use_naive_des);
    use_clustering_des.description =
      "Choose to use clustering in the mutual information calculation";
    declare_parameter("use_clustering", false, use_clustering_des);
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
    use_clustering_ = get_parameter("use_clustering").as_bool();
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

    pub_goal_marker_ = create_publisher<visualization_msgs::msg::Marker>("/vp", 10);

    clusters_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/clusters",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::TransientLocal));
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    goal_publisher_->on_activate();
    map_with_frontiers_pub_->on_activate();
    pub_goal_marker_->on_activate();
    clusters_pub_->on_activate();

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
    pub_goal_marker_->on_deactivate();
    clusters_pub_->on_deactivate();
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
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  pub_goal_marker_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  clusters_pub_;

  // Services for action node
  rclcpp::CallbackGroup::SharedPtr nav_to_pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_nav_callback_group_;

  rclcpp::Client<nav_client_cpp::srv::NavToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cancel_nav_client_;

  // Data objects
  nav_msgs::msg::OccupancyGrid map_data_;
  std::vector<std::pair<int, int>> frontiers_;
  std::vector<std::pair<int, int>> sampled_frontiers_;
  std::vector<std::pair<float, float>> frontier_cluster_centroids_;
  std::vector<double> entropies_;
  std::vector<int> unknowns_;
  std::pair<double, double> robot_position_;
  std::optional<std::pair<double, double>> robot_vp_position_;
  std::set<int> active_marker_ids_;
  std::map<int, std::vector<std::pair<int, int>>> all_clusters_;

  double viewpoint_depth_;
  bool is_sim_;
  double entropy_radius_ = 2.5;
  bool use_naive_;
  bool use_clustering_;
  bool eval_cluster_size_ = false;
  bool use_sampling_;
  bool use_entropy_calc_;
  int sampling_threshold;
  bool use_action_client_;
  int best_frontier_idx_;
  double robot_radius_;

  // Buffer objects
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

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
        auto [index, entropy] = FrontierHelper::bestEntropyIndexScore(entropies_);
        best_frontier_idx_ = index;
        RCLCPP_INFO(
          get_logger(), "Selecting frontier %d, with entropy reduction %f", best_frontier_idx_,
          entropy);
      } else {
        // Erase most recently selected frontier and its score
        unknowns_.erase(unknowns_.begin() + best_frontier_idx_);
        frontiers.erase(frontiers.begin() + best_frontier_idx_);

        // Find and return next best score and index
        auto [index, score] = FrontierHelper::bestUnknownsIndexScore(unknowns_);
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
      robot_vp_position_ = getRobotViewpoint();
      if (robot_vp_position_) {
        findFrontiers();
        cleanupFrontiers();

        if (use_clustering_) {
          all_clusters_ = clusterFrontiers(frontiers_, 20.0, 5);
          publishClustersAsMarkers(all_clusters_, "/map", map_data_.info.resolution, 0.06);
        }

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

      if (FrontierHelper::isPositionOutsideMap(map_data_, x_adj, y_adj)) {
        x_adj = 0.0;
        y_adj = 0.0;
      }
      publishViewpoint(x_adj, y_adj);

      RCLCPP_DEBUG(get_logger(), "Got new robot position at x,y: %f, %f", x, y);
      RCLCPP_DEBUG(get_logger(), "Got new viewpoint at %f, %f", x_adj, y_adj);
      return std::make_optional(std::make_pair(x_adj, y_adj));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Error getting transform: %s", ex.what());
      return std::nullopt;
    }
  }

  void publishViewpoint(double x, double y)
  {
    visualization_msgs::msg::Marker viewpoint;
    viewpoint.header.frame_id = "/map";
    viewpoint.header.stamp = get_clock()->now();
    viewpoint.id = 0;
    viewpoint.type = visualization_msgs::msg::Marker::CYLINDER;
    viewpoint.action = visualization_msgs::msg::Marker::ADD;
    viewpoint.pose.position.x = x;
    viewpoint.pose.position.y = y;
    viewpoint.pose.position.z = 0.01;
    // viewpoint.pose.orientation = orientation;
    viewpoint.scale.x = 0.05;
    viewpoint.scale.y = 0.05;
    viewpoint.scale.z = 0.05;
    viewpoint.color.r = 1.0f;
    viewpoint.color.g = 0.5f;
    viewpoint.color.b = 0.0f;
    viewpoint.color.a = 1.0;

    pub_goal_marker_->publish(viewpoint);
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
        if (data[idx] == -1 && FrontierHelper::hasFreeNeighbor(map_data_, x, y)) {
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

  void cleanupFrontiers()
  {
    std::vector<std::pair<int, int>> valid_frontiers;
    const auto & data = map_data_.data;

    for (const auto & frontier : frontiers_) {
      int idx = frontier.second * map_data_.info.width + frontier.first;
      if (data[idx] == -1 &&
        FrontierHelper::hasFreeNeighbor(
          map_data_, frontier.first,
          frontier.second) && !tooClose(frontier))
      {
        valid_frontiers.push_back(frontier);
      }
    }
    frontiers_ = valid_frontiers;
  }

  void publishClustersAsMarkers(
    const std::map<int, std::vector<std::pair<int, int>>> & clusters,
    const std::string & frame_id,
    float resolution,
    float z_level = 0.0)
  {

    // Clear old markers
    clearOldMarkers(frame_id);

    visualization_msgs::msg::MarkerArray marker_array;

    // Iterate through clusters and create markers
    for (const auto & [cluster_id, points] : clusters) {
      // Create POINTS marker for the cluster
      visualization_msgs::msg::Marker points_marker;
      points_marker.header.frame_id = frame_id;
      points_marker.header.stamp = rclcpp::Clock().now();
      points_marker.ns = "cluster_points";
      points_marker.id = cluster_id;  // Unique ID per cluster
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.action = visualization_msgs::msg::Marker::ADD;
      points_marker.scale.x = resolution;  // Size of points
      points_marker.scale.y = resolution;

      // Randomly generate a unique color for each cluster
      points_marker.color.r = static_cast<float>(std::rand() % 255) / 255.0;
      points_marker.color.g = static_cast<float>(std::rand() % 255) / 255.0;
      points_marker.color.b = static_cast<float>(std::rand() % 255) / 255.0;
      points_marker.color.a = 1.0;  // Fully opaque

      // Add all points in the cluster and create a centroid
      geometry_msgs::msg::Point centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = z_level;

      for (const auto & point : points) {
        auto [world_x, world_y] = cellToWorld(point);
        geometry_msgs::msg::Point p;
        p.x = world_x;
        p.y = world_y;
        p.z = z_level;      // Optional z-level (e.g., ground plane)
        points_marker.points.push_back(p);

        // Accumulate centroid
        centroid.x += world_x;
        centroid.y += world_y;
      }

      // Finalize centroid by averaging
      centroid.x /= points.size();
      centroid.y /= points.size();

      // Add the cluster centroids to the vector for later use
      frontier_cluster_centroids_.emplace_back(centroid.x, centroid.y);

      marker_array.markers.push_back(points_marker);

      // Create TEXT_VIEW_FACING marker for the cluster ID
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = rclcpp::Clock().now();
      text_marker.ns = "cluster_label";
      text_marker.id = cluster_id + clusters.size(); // Ensure unique ID
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.scale.z = resolution * 3.0;  // Text size
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;  // Fully opaque
      text_marker.pose.position = centroid;
      text_marker.pose.position.z += z_level + 0.1; // Slightly above points
      text_marker.text = std::to_string(cluster_id);

      marker_array.markers.push_back(text_marker);

      // Track the marker ID
      active_marker_ids_.insert(cluster_id);
    }

    RCLCPP_INFO(
      get_logger(), "Publishing MarkerArray with %ld markers",
      marker_array.markers.size());
    // Publish the marker array
    clusters_pub_->publish(marker_array);
  }

  void clearOldMarkers(const std::string & frame_id)
  {
    frontier_cluster_centroids_.clear(); // clear out old centroids
    visualization_msgs::msg::MarkerArray clear_array;

    // Iterate over the list of previously published markers
    for (int id : active_marker_ids_) {
      // delete point markers
      visualization_msgs::msg::Marker marker_points;
      marker_points.header.frame_id = frame_id;
      marker_points.header.stamp = rclcpp::Clock().now();
      marker_points.ns = "cluster_points";    // Namespace for the markers
      marker_points.id = id;                  // Use the active marker ID
      marker_points.action = visualization_msgs::msg::Marker::DELETE;    // Set action to DELETE
      clear_array.markers.push_back(marker_points);

      // delete text markers
      visualization_msgs::msg::Marker marker_text;
      marker_text.header.frame_id = frame_id;
      marker_text.header.stamp = rclcpp::Clock().now();
      marker_text.ns = "cluster_label";    // Namespace for the markers
      marker_text.id = id + active_marker_ids_.size();                  // Use the active marker ID
      marker_text.action = visualization_msgs::msg::Marker::DELETE;    // Set action to DELETE
      clear_array.markers.push_back(marker_text);
    }

    // Publish the deletion markers
    clusters_pub_->publish(clear_array);

    // Clear the active_marker_ids_ list to reset tracking
    active_marker_ids_.clear();
  }

  // Main clustering function
  std::map<int, std::vector<std::pair<int, int>>> clusterFrontiers(
    const std::vector<std::pair<int, int>> & frontiers, float eps,
    int min_samples)
  {
    cv::Mat points;
    for (const auto & f : frontiers) {
      points.push_back(cv::Vec2f(f.first, f.second));
    }

    auto labels = FrontierHelper::performDBSCAN(points, eps, min_samples);

    // Organize clusters
    std::map<int, std::vector<std::pair<int, int>>> clusters;
    clusters.clear();
    for (size_t i = 0; i < frontiers.size(); ++i) {
      int label = labels.at(i);
      if (label >= 0) {      // Ignore noise
        clusters[label].emplace_back(frontiers.at(i));
      }
    }

    // Filter out small clusters
    auto filtered_clusters = filterClusters(clusters, min_samples);

    RCLCPP_INFO(get_logger(), "Number of frontiers: %ld", frontiers.size());
    // Print cluster information
    for (const auto & [id, cluster] : filtered_clusters) {
      std::cout << "Cluster " << id << " has " << cluster.size() << " points\n";
    }

    RCLCPP_INFO(get_logger(), "Number of clusters: %ld", filtered_clusters.size());
    return filtered_clusters;
  }

  // Filter out clusters smaller than min_samples
  std::map<int, std::vector<std::pair<int, int>>> filterClusters(
    const std::map<int, std::vector<std::pair<int, int>>> & clusters,
    int min_samples)
  {
    std::map<int, std::vector<std::pair<int, int>>> filtered_clusters;
    for (const auto & [id, cluster] : clusters) {
      if (static_cast<int>(cluster.size()) >= min_samples) {
        filtered_clusters[id] = cluster;
      }
    }
    return filtered_clusters;
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

    if (!use_action_client_) {
      publishToNav2PlannerServer(selectGoal());
    } else {
      publishToNav2ActionClient(selectGoal());
    }
  }

  std::pair<int, int> selectGoal()
  {
    std::pair<int, int> goal_frontier;
    if (use_naive_ == true) {
      // Using naive algorithm
      if (use_sampling_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
        // Using sampling method if selected to make direct comparison
        sampled_frontiers_ = FrontierHelper::sampleRandomFrontiers(frontiers_, sampling_threshold);
        goal_frontier = *std::min_element(
          sampled_frontiers_.begin(), sampled_frontiers_.end(),
          [this](const auto & f1, const auto & f2)
          {
            return distanceToRobotVP(f1) < distanceToRobotVP(f2);
          });
      } else {
        // Default method
        goal_frontier = *std::min_element(
          frontiers_.begin(), frontiers_.end(), [this](const auto & f1, const auto & f2)
          {
            return distanceToRobotVP(f1) < distanceToRobotVP(f2);
          });
      }
    } else if (use_clustering_) {
      if (eval_cluster_size_) {
        std::vector<std::pair<int, int>> cell_centroids = FrontierHelper::getCentroidCells(
        map_data_,
        frontier_cluster_centroids_);
        int largest_cluster_id = FrontierHelper::findLargestCluster(all_clusters_);
        // put code here to remove if too close or other things.
        goal_frontier = cell_centroids.at(largest_cluster_id);
      } else {
        std::vector<std::pair<int, int>> cell_centroids = FrontierHelper::getCentroidCells(
        map_data_,
        frontier_cluster_centroids_);
        goal_frontier = bestScoringFrontier(cell_centroids);
      }
    } else if (use_sampling_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
      sampled_frontiers_ = FrontierHelper::sampleRandomFrontiers(frontiers_, sampling_threshold);
      goal_frontier = bestScoringFrontier(sampled_frontiers_);
      RCLCPP_DEBUG(get_logger(), "Using sampling to get best frontier");
    } else {
      goal_frontier = bestScoringFrontier(frontiers_);
    }

    return goal_frontier;
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
      modified_map.data.at(idx) = 50;       // Mark frontiers in the map
    }

    map_with_frontiers_pub_->publish(modified_map);
    RCLCPP_DEBUG(get_logger(), "Published map with highlighted frontiers.");
  }

  double distanceToRobot(const std::pair<int, int> & frontier)
  {
    auto [fx, fy] = cellToWorld(frontier);
    auto [rx, ry] = robot_position_;
    return std::hypot(fx - rx, fy - ry);
  }

  double distanceToRobotVP(const std::pair<int, int> & frontier)
  {
    auto [fx, fy] = cellToWorld(frontier);
    double rx = 0.0, ry = 0.0; // Initialize rx and ry
    if (robot_vp_position_) { // Assign values if robot_vp_position_ is set
      std::tie(rx, ry) = *robot_vp_position_;   // Dereferencing pointer :O
    }
    return std::hypot(fx - rx, fy - ry);
  }

  std::pair<double, double> cellToWorld(const std::pair<int, int> & cell)
  {
    double world_x = map_data_.info.origin.position.x + (cell.first * map_data_.info.resolution);
    double world_y = map_data_.info.origin.position.y + (cell.second * map_data_.info.resolution);
    return {world_x, world_y};
  }

  std::pair<int, int> bestScoringFrontier(const std::vector<std::pair<int, int>> & frontiers)
  {
    double total_entropy;
    // Establish baseline and reset for preferred scoring approach
    if (use_entropy_calc_) {
      total_entropy = FrontierHelper::calculateMapEntropy(map_data_.data);
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
      int unknowns = FrontierHelper::countUnknownCellsWithinRadius(map_data_, idx, entropy_radius_);

      if (use_entropy_calc_) {
        // calculate current reduced entropy and place in list
        entropies_.emplace_back(
          total_entropy - (unknowns * FrontierHelper::calculateEntropy(-1)) +
          (unknowns * FrontierHelper::calculateEntropy(0)));
      } else {
        unknowns_.emplace_back(unknowns);
      }
    }

    if (use_entropy_calc_) {
      // Find and return best entropy and index
      auto [index, entropy] = FrontierHelper::bestEntropyIndexScore(entropies_);
      best_frontier_idx_ = index;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with entropy reduction %f", best_frontier_idx_,
        entropy);
    } else {
      // Find and return best score and index
      auto [index, score] = FrontierHelper::bestUnknownsIndexScore(unknowns_);
      best_frontier_idx_ = index;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with best score %d", best_frontier_idx_,
        score);
    }

    return frontiers.at(best_frontier_idx_);
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
