#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include <vector>
#include <utility> // For std::pair
#include <map>
#include <cmath>
#include <opencv2/core.hpp>
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
};

#endif // DBSCAN_HPP
