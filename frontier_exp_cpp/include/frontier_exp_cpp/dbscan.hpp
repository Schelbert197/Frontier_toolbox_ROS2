#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include "frontier_exp_cpp/frontier_helper.hpp"
#include <opencv2/core.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <set>


// @class DBSCAN
/// @brief A utility class for frontier clustering, providing helper functions for DBSCAN.
class DBSCAN
{
public:
  using Cell = std::pair<int, int>;
  using Coord = std::pair<double, double>;

  /// @brief A way to manage all objects associated with the clusters
  struct ClusterObj
  {
    /// @brief A map of all of the cluster ID's and associated points.
    std::map<int, std::vector<Cell>> clusters;

    /// @brief A vector of pairs containing centroids in world coordinates.
    std::vector<Coord> world_centroids;

    /// @brief A vector of pairs containing centroids in cell coordinates.
    std::vector<Cell> cell_centroids;
  };

  /// @brief Main function to create clusters.
  /// @param frontiers The vector of frontier cells to be clustered.
  /// @param eps Epsilon: The maximum distance to consider two points as neighbors.
  /// @param min_samples The minimum number of neighbors required to make a point a core point.
  /// @return A map of cluster IDs and their associated clustered points.
  static std::map<int, std::vector<DBSCAN::Cell>> clusterFrontiers(
  const std::vector<Cell> & frontiers, float eps,
  int min_samples);

  /// @brief Filters out clusters smaller then the minimum number of samples
  /// @param clusters The existing map of clustered frontiers
  /// @param min_samples The minimum number of neighbors required to make a point a core point.
  /// @return A smaller map without the noise data.
  static std::map<int, std::vector<DBSCAN::Cell>> filterClusters(
  const std::map<int, std::vector<Cell>> & clusters,
  int min_samples);

  /// @brief Performs DBSCAN clustering on a set of points.
  /// @param points A matrix of points (each row represents a point).
  /// @param eps The maximum distance between points to be considered neighbors.
  /// @param min_samples The minimum number of points to form a dense region (cluster).
  /// @return A vector of labels where each element corresponds to the cluster ID of a point.
  ///         Points labeled as -1 are considered noise.
  static std::vector<int> performDBSCAN(
    const cv::Mat & points,
    double eps,
    int min_samples);

  /// @brief Merges adjacent clusters in a map of clusters.
  /// @param clusters A map of cluster IDs to vectors of (x, y) cell coordinates.
  /// @return A new map with adjacent clusters merged.
  static std::map<int, std::vector<Cell>> mergeAdjacentClusters(
    const std::map<int, std::vector<Cell>> & clusters);

  /// @brief Finds the cluster with the largest number of associated points.
  /// @param clusters The hashmap of id's and their associated clustered points.
  /// @return An integer representing the largest cluster.
  static int findLargestCluster(
    const DBSCAN::ClusterObj & cluster_obj, const FrontierHelper::BannedAreas & banned,
    const nav_msgs::msg::OccupancyGrid & map_data);

  /// @brief Finds the index of the second largest cluster.
  /// @param clusters A map of cluster indices to their associated points.
  /// @return The index of the second largest cluster, or -1 if there are fewer than 2 clusters.
  static int findSecondLargestCluster(
    const std::map<int, std::vector<Cell>> & clusters);
};

#endif // DBSCAN_HPP
