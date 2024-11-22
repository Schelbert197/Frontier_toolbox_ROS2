#ifndef FRONTIER_HELPER_HPP
#define FRONTIER_HELPER_HPP

#include <vector>
#include <utility> // For std::pair
#include <map>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <iostream>


/// @class FrontierHelper
/// @brief A utility class for frontier exploration, providing helper functions for map analysis.
class FrontierHelper
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

  struct BannedAreas
  {
    /// @brief A radius [m] around a cell to consider "banned".
    double radius;

    /// @brief The list of coords that are to be avoided.
    std::vector<Coord> coords;
  };

  /// @brief Stage a new area to be banned, or add to an existing area within radius
  /// @param cell Location to be evaluated for banishment staging.
  /// @param staged Hashmap of the existing places staged for banishment.
  /// @param rad The radius in which to asses whether a location is near an existing cell.
  static std::map<Coord, int> stageBanned(
    const Cell & cell, std::map<Coord, int> & staged,
    double rad,
    const nav_msgs::msg::OccupancyGrid & map_data);

  /// @brief Takes items from staged for banishment and officially banishes them.
  /// @param staged
  /// @param banned
  /// @return The new banned
  static BannedAreas addBanned(std::map<Coord, int> & staged, BannedAreas & banned);

  /// @brief Checks if a given position is outside the bounds of the map.
  /// @param map The occupancy grid map.
  /// @param robot_x The x-coordinate of the robot in world coordinates.
  /// @param robot_y The y-coordinate of the robot in world coordinates.
  /// @return True if the position is outside the map, false otherwise.
  static bool isPositionOutsideMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const double & robot_x, const double & robot_y);

  /// @brief Checks if a map cell has at least one free neighboring cell.
  /// @param map_data The occupancy grid map.
  /// @param x The x-coordinate of the cell.
  /// @param y The y-coordinate of the cell.
  /// @return True if a free neighbor exists, false otherwise.
  static bool hasFreeNeighbor(
    nav_msgs::msg::OccupancyGrid & map_data,
    int x, int y);

  /// @brief Checks if the line between two points is occluded.
  /// @param x1 The x-coordinate of the start point.
  /// @param y1 The y-coordinate of the start point.
  /// @param x2 The x-coordinate of the end point.
  /// @param y2 The y-coordinate of the end point.
  /// @param width The width of the map.
  /// @param map_data The occupancy grid data.
  /// @return True if the line is occluded, false otherwise.
  static bool occluded(
    int x1, int y1, int x2, int y2, int width,
    const std::vector<int8_t> & map_data);

  /// @brief Calculates the entropy of a map cell value.
  /// @param cell_value The occupancy value of the cell (-1: unknown, 0: free, 100: occupied).
  /// @return The entropy value for the given cell.
  static double calculateEntropy(int cell_value);

  /// @brief Counts the number of unknown cells within a given radius of a specified index.
  /// @param map_data The occupancy grid map data.
  /// @param index The index of the center cell.
  /// @param rad The radius within which to count unknown cells.
  /// @return The count of unknown cells within the radius.
  static int countUnknownCellsWithinRadius(
    nav_msgs::msg::OccupancyGrid & map_data,
    int index,
    double rad);

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

  /// @brief Selects the least entropy from a list and returns its index and value.
  /// @param entropies A vector containing entropy values.
  /// @return A pair containing the index of the best frontier and the corresponding entropy value.
  static std::pair<int, double> bestEntropyIndexScore(const std::vector<double> & entropies);

  /// @brief Selects the most converted unknowns from a list and returns its index and value.
  /// @param unknowns A vector containing the number of unknown cells.
  /// @return A pair containing the index of the best frontier and the corresponding number of unknowns.
  static Cell bestUnknownsIndexScore(const std::vector<int> & unknowns);

  /// @brief Calculates the entropy of a map based on its data.
  /// @param map_data A vector containing the occupancy grid data.
  /// @return The total entropy of the map.
  static double calculateMapEntropy(const std::vector<int8_t> & map_data);

  /// @brief Randomly samples a specified number of frontiers from a given list.
  /// @param frontiers The list of frontier coordinates (x, y).
  /// @param sample_size The number of frontiers to sample.
  /// @return A vector of sampled frontiers, up to the specified sample size.
  static std::vector<Cell> sampleRandomFrontiers(
    const std::vector<Cell> & frontiers,
    size_t sample_size);

  /// @brief Returns a grid cell in world coordinates.
  /// @param cell The grid cell to be converted.
  /// @param map_data The occupancy grid map data.
  /// @return The x,y position of the cell in meter from the map origin.
  static std::pair<double, double> cellToWorld(
    const Cell & cell,
    const nav_msgs::msg::OccupancyGrid & map_data);

  /// @brief Translates the centroid coordinates from the world meters to map cells.
  /// @param map The occupancy grid map.
  /// @param centroids The list of centroids to convert.
  /// @return A vector of centroids as cells in the map.
  static std::vector<Cell> getCentroidCells(
    const nav_msgs::msg::OccupancyGrid & map,
    std::vector<Coord> centroids);

  /// @brief Finds the cluster with the largest number of associated points.
  /// @param clusters The hashmap of id's and their associated clustered points.
  /// @return An integer representing the largest cluster.
  static int findLargestCluster(
    const std::map<int, std::vector<Cell>> & clusters);

  /// @brief Finds the index of the second largest cluster.
  /// @param clusters A map of cluster indices to their associated points.
  /// @return The index of the second largest cluster, or -1 if there are fewer than 2 clusters.
  static int findSecondLargestCluster(
    const std::map<int, std::vector<Cell>> & clusters);
};

#endif // FRONTIER_HELPER_HPP
