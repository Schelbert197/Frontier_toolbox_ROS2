// Copyright 2024 Srikanth Schelbert.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief A node to run various frontier exploration algorithms.

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
#include "frontier_exp_cpp/dbscan.hpp"
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
    auto eval_cluster_size_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto free_edge_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_action_client_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto viewpoint_depth_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto robot_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto entropy_calc_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_sampling_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto sampling_threshold_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto banning_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto stuck_tolerance_des = rcl_interfaces::msg::ParameterDescriptor{};

    // Parameter Descriptions and Declarations
    use_naive_des.description = "Choose to use naive frontier selection or mutual information";
    declare_parameter("use_naive", false, use_naive_des);
    use_clustering_des.description =
      "Choose to use clustering to organize frontiers using DBSCAN";
    declare_parameter("use_clustering", true, use_clustering_des);
    eval_cluster_size_des.description =
      "Choose to select goal cluster by cluster size";
    declare_parameter("eval_cluster_size", false, eval_cluster_size_des);
    free_edge_des.description =
      "Choose to consider an unoccupied map edge as a frontier";
    declare_parameter("consider_free_edge", false, free_edge_des);
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
    banning_radius_des.description =
      "The radius around which to consider a goal position banished from being a goal.";
    declare_parameter("banning_radius", 1.0, banning_radius_des);
    stuck_tolerance_des.description =
      "The tolerance which to consider the robot to be stuck (hasn't moved more than x[m]).";
    declare_parameter("stuck_tolerance", 0.001, stuck_tolerance_des);

    // Get Parameters
    is_sim_ = get_parameter("use_sim_time").as_bool();
    use_naive_ = get_parameter("use_naive").as_bool();
    use_clustering_ = get_parameter("use_clustering").as_bool();
    eval_cluster_size_ = get_parameter("eval_cluster_size").as_bool();
    consider_free_edge_ = get_parameter("consider_free_edge").as_bool();
    use_action_client_ = get_parameter("use_action_client_node").as_bool();
    viewpoint_depth_ = get_parameter("viewpoint_depth").as_double();
    entropy_radius_ = get_parameter("entropy_radius").as_double();
    robot_radius_ = get_parameter("robot_radius").as_double();
    use_entropy_calc_ = get_parameter("use_entropy_calc").as_bool();
    use_sampling_ = get_parameter("use_sampling").as_bool();
    sampling_threshold = get_parameter("sampling_threshold").as_int();
    banned_.radius = get_parameter("banning_radius").as_double();
    EPSILON = get_parameter("stuck_tolerance").as_double();

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

  // For simplification
  using Cell = FrontierHelper::Cell;
  using Coord = FrontierHelper::Coord;

  // Data objects
  nav_msgs::msg::OccupancyGrid map_data_;
  std::vector<Cell> frontiers_;
  std::vector<Cell> sampled_frontiers_;
  std::vector<double> entropies_;
  std::vector<int> unknowns_;
  Coord robot_position_;
  Coord last_robot_position_;
  std::optional<Coord> robot_vp_position_;
  std::set<int> active_marker_ids_;
  DBSCAN::ClusterObj my_clusters_;
  FrontierHelper::BannedAreas banned_;

  double viewpoint_depth_;
  bool is_sim_;
  double entropy_radius_ = 2.5;
  bool use_naive_;
  bool use_clustering_;
  bool eval_cluster_size_;
  bool consider_free_edge_;
  bool use_sampling_;
  bool use_entropy_calc_;
  int sampling_threshold;
  bool use_action_client_;
  int best_frontier_idx_;
  double robot_radius_;
  double EPSILON = 0.001;

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

  void selectNextBest(std::vector<Cell> & frontiers)
  {
    if (use_clustering_) {
      int s_largest_cluster_id = DBSCAN::findSecondLargestCluster(my_clusters_.clusters);
      publishToNav2ActionClient(my_clusters_.cell_centroids.at(s_largest_cluster_id));
      RCLCPP_INFO(get_logger(), "Replanning for second largest cluster");
    } else {
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
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_data_ = *msg;

    auto current_state = this->get_current_state();
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      // Get the robot's current viewpoint position from the transform
      robot_vp_position_ = getRobotViewpoint();
      if (robot_vp_position_.has_value()) {
        frontiers_ = FrontierHelper::findFrontiers(map_data_, consider_free_edge_);
        frontiers_ = cleanupFrontiers(frontiers_);

        if (use_clustering_) {
          my_clusters_.clusters = DBSCAN::clusterFrontiers(frontiers_, 6.0, 5);
          publishClustersAsMarkers(my_clusters_.clusters, "/map", map_data_.info.resolution, 0.06);
          updateClusters(); // sets cluster centroid array
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

  std::optional<Coord> getRobotViewpoint()
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

  std::vector<Cell> cleanupFrontiers(const std::vector<Cell> & frontiers)
  {
    std::vector<Cell> filtered;
    for (const auto & frontier : frontiers) {
      if (!tooClose(frontier)) {
        filtered.push_back(frontier);
      }
    }
    // Convert the vector to a string to print out if needed
    std::stringstream ss;
    ss << "Vector contents: ";
    for (const auto & pair : filtered) {
      ss << "(" << pair.first << ", " << pair.second << ") ";
    }
    // Print the vector contents of the frontier vector
    RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
    return filtered;
  }


  void publishClustersAsMarkers(
    const std::map<int, std::vector<Cell>> & clusters,
    const std::string & frame_id,
    float resolution,
    float z_level = 0.0)
  {
    // Clear old markers
    clearOldMarkers(frame_id);

    // Iterate through clusters and create markers
    visualization_msgs::msg::MarkerArray marker_array;
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
        auto [world_x, world_y] = FrontierHelper::cellToWorld(point, map_data_);
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
      my_clusters_.world_centroids.emplace_back(centroid.x, centroid.y);

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
    my_clusters_.world_centroids.clear(); // clear out old centroids
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

  bool tooClose(const Cell & frontier)
  {
    return FrontierHelper::cellCoordDistance(frontier, robot_position_, map_data_) <= robot_radius_;
  }

  bool stuck() const
  {
    return std::abs(last_robot_position_.first - robot_position_.first) < EPSILON &&
           std::abs(last_robot_position_.second - robot_position_.second) < EPSILON;
  }

  void publishGoalFrontier()
  {
    if (frontiers_.empty()) {
      RCLCPP_INFO(get_logger(), "No frontiers available.");
      return;
    } else if (use_clustering_ && my_clusters_.cell_centroids.size() < 1) {
      RCLCPP_INFO(get_logger(), "No valid clusters available.");
      return;
    } // Exits if there aren't any goals to visit

    if (!use_action_client_) {
      publishToNav2PlannerServer(selectGoal());
    } else {
      publishToNav2ActionClient(selectGoal());
    }
  }

  Cell selectGoal()
  {
    Cell goal_frontier; // Define goal frontier
    if (use_naive_) {
      goal_frontier = selectNaiveGoal();
    } else if (use_clustering_) {
      goal_frontier = selectClusterGoal();
    } else if (use_sampling_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
      goal_frontier = selectSampledGoal();
    } else {
      goal_frontier = bestScoringFrontier(frontiers_);
    }

    last_robot_position_ = robot_position_;
    return goal_frontier;
  }

  // Helper function to update clusters
  void updateClusters()
  {
    my_clusters_.cell_centroids = FrontierHelper::getCentroidCells(
      map_data_, my_clusters_.world_centroids);
    RCLCPP_INFO(
      get_logger(),
      "Number of clusters: %ld\nNumber of centroids: %ld",
      my_clusters_.clusters.size(), my_clusters_.cell_centroids.size());
  }

  // Helper function to handle naive goal selection
  Cell selectNaiveGoal()
  {
    if (use_clustering_) {
      return FrontierHelper::selectByDistance(
        my_clusters_.cell_centroids, robot_vp_position_.value(),
        map_data_);
    } else if (use_sampling_ && static_cast<int>(frontiers_.size()) > sampling_threshold) {
      return FrontierHelper::selectByDistance(
        FrontierHelper::sampleRandomFrontiers(
          frontiers_,
          sampling_threshold), robot_vp_position_.value(), map_data_);
    }
    return FrontierHelper::selectByDistance(frontiers_, robot_vp_position_.value(), map_data_);
  }

  // Helper function to handle clustered goal selection
  Cell selectClusterGoal()
  {
    if (eval_cluster_size_) {
      int largest_cluster_id = DBSCAN::findLargestCluster(my_clusters_, banned_, map_data_);
      RCLCPP_INFO(
        get_logger(), "selecting cluster %d with size: %ld", largest_cluster_id,
        my_clusters_.clusters.size());
      return replanIfStuck(my_clusters_.cell_centroids.at(largest_cluster_id));
    }
    Cell goal_frontier = bestScoringFrontier(my_clusters_.cell_centroids);
    RCLCPP_INFO(
      get_logger(),
      "last_robot_position: %f, %f \nCurrent Robot Position: %f, %f",
      last_robot_position_.first, last_robot_position_.second,
      robot_position_.first, robot_position_.second);
    return replanIfStuck(goal_frontier);
  }

  // Helper function to handle sampled goal selection
  Cell selectSampledGoal()
  {
    sampled_frontiers_ = FrontierHelper::sampleRandomFrontiers(frontiers_, sampling_threshold);
    RCLCPP_DEBUG(get_logger(), "Using sampling to get best frontier");
    return bestScoringFrontier(sampled_frontiers_);
  }

  void publishToNav2ActionClient(const Cell & goal_frontier)
  {
    auto goal_pose = std::make_shared<nav_client_cpp::srv::NavToPose::Request>();
    std::tie(goal_pose->x, goal_pose->y) = FrontierHelper::cellToWorld(goal_frontier, map_data_);
    goal_pose->theta = 0.0;
    RCLCPP_INFO(
      get_logger(), "Publishing goal at %f, %f", goal_pose->x, goal_pose->y);
    auto future_result = nav_to_pose_client_->async_send_request(goal_pose);
  }

  void publishToNav2PlannerServer(const Cell & goal_frontier)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    std::tie(goal_pose.pose.position.x, goal_pose.pose.position.y) = FrontierHelper::cellToWorld(
      goal_frontier, map_data_);
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

  Cell replanIfStuck(Cell & curr_goal)
  {
    if (stuck()) {
      Cell goal_frontier =
        my_clusters_.cell_centroids.at(
        DBSCAN::findSecondLargestCluster(
          my_clusters_.
          clusters));
      RCLCPP_INFO(get_logger(), "Going to second largest cluster because robot hasn't moved");
      return goal_frontier;
    } else {
      return curr_goal;
    }
  }

  Cell bestScoringFrontier(const std::vector<Cell> & frontiers)
  {
    Cell goal;

    if (use_entropy_calc_) {
      auto [goal_frontier, entropies, idx] = FrontierHelper::scoreByEntropy(
        frontiers, map_data_,
        entropy_radius_,
        banned_);
      goal = goal_frontier;
      entropies_ = entropies;
      best_frontier_idx_ = idx;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with entropy reduction %f", best_frontier_idx_,
        entropies.at(idx));
    } else {
      auto [goal_frontier, unknowns, idx] = FrontierHelper::scoreByFlipCount(
        frontiers, map_data_,
        entropy_radius_,
        banned_);
      goal = goal_frontier;
      unknowns_ = unknowns;
      best_frontier_idx_ = idx;
      RCLCPP_INFO(
        get_logger(), "Selecting frontier %d, with best score %d", best_frontier_idx_,
        unknowns.at(idx));
    }

    return goal;
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
