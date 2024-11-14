#ifndef FRONTIER_HELPER_HPP
#define FRONTIER_HELPER_HPP

#include <vector>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>


/**
 * @class FrontierHelper
 * @brief A utility class for frontier exploration, providing helper functions for map analysis.
 */
class FrontierHelper
{
public:
  /**
   * @brief Checks if a given position is outside the bounds of the map.
   * @param map The occupancy grid map.
   * @param robot_x The x-coordinate of the robot in world coordinates.
   * @param robot_y The y-coordinate of the robot in world coordinates.
   * @return True if the position is outside the map, false otherwise.
   */
  static bool isPositionOutsideMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const double & robot_x, const double & robot_y);

  /**
   * @brief Checks if a map cell has at least one free neighboring cell.
   * @param map_data The occupancy grid map.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return True if a free neighbor exists, false otherwise.
   */
  static bool hasFreeNeighbor(
    nav_msgs::msg::OccupancyGrid & map_data,
    int x, int y);

  /**
   * @brief Checks if the line between two points is occluded.
   * @param x1 The x-coordinate of the start point.
   * @param y1 The y-coordinate of the start point.
   * @param x2 The x-coordinate of the end point.
   * @param y2 The y-coordinate of the end point.
   * @param width The width of the map.
   * @param map_data The occupancy grid data.
   * @return True if the line is occluded, false otherwise.
   */
  static bool occluded(
    int x1, int y1, int x2, int y2, int width,
    const std::vector<int8_t> & map_data);

  /**
   * @brief Calculates the entropy of a map cell value.
   * @param cell_value The occupancy value of the cell (-1: unknown, 0: free, 100: occupied).
   * @return The entropy value for the given cell.
   */
  static double calculateEntropy(int cell_value);

  /**
 * @brief Counts the number of unknown cells within a given radius of a specified index.
 * @param map_data The occupancy grid map data.
 * @param index The index of the center cell.
 * @param rad The radius within which to count unknown cells.
 * @return The count of unknown cells within the radius.
 */
  static int countUnknownCellsWithinRadius(
    nav_msgs::msg::OccupancyGrid & map_data,
    int index,
    double rad);

  /**
   * @brief Performs DBSCAN clustering on a set of points.
   * @param points A matrix of points (each row represents a point).
   * @param eps The maximum distance between points to be considered neighbors.
   * @param min_samples The minimum number of points to form a dense region (cluster).
   * @return A vector of labels where each element corresponds to the cluster ID of a point.
   *         Points labeled as -1 are considered noise.
   */
  static std::vector<int> performDBSCAN(
    const cv::Mat & points,
    float eps,
    int min_samples);
};

#endif // FRONTIER_HELPER_HPP
