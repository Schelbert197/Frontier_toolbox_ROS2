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

  struct BannedAreas
  {
    /// @brief A radius [m] around a cell to consider "banned".
    double radius;

    /// @brief The list of coords that are to be avoided.
    std::vector<Coord> coords;
  };

  /// @brief Finds all "frontiers" in an OccupancyGrid map.
  /// @param map_data
  /// @param consider_free_edge
  /// @return A vector of std::pair<int, int> grid cells of frontiers.
  static std::vector<Cell> findFrontiers(
    const nav_msgs::msg::OccupancyGrid & map_data,
    bool consider_free_edge);

  /// @brief Stage a new area to be banned, or add to an existing area within radius
  /// @param cell Location to be evaluated for banishment staging.
  /// @param staged Hashmap of the existing places staged for banishment.
  /// @param rad The radius in which to asses whether a location is near an existing cell.
  static std::map<Coord, int> stageBanned(
    const Cell & cell, std::map<Coord, int> & staged,
    double rad,
    const nav_msgs::msg::OccupancyGrid & map_data);

  /// @brief Takes items from staged for banishment and officially banishes them.
  /// @param staged The map of staged locations to be banned.
  /// @param banned The struct of banned goal locations.
  /// @return The new banned locations struct.
  static BannedAreas addBanned(std::map<Coord, int> & staged, BannedAreas & banned);

  /// @brief Checks if a location if a goal location is considered in a banned area
  /// @param cell The cell to be evaluated.
  /// @param banned The struct with the list of banned locations.
  /// @return A true or false whether the location is near a banned location.
  static bool identifyBanned(
    const Cell & cell, const BannedAreas & banned,
    const nav_msgs::msg::OccupancyGrid & map_data);

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
    const nav_msgs::msg::OccupancyGrid & map_data,
    int x, int y);

  /// @brief Checks if a map cell is a frontier by being empty and map edge.
  /// @param map_data Occupancy grid map
  /// @param x The x-coordinate of the cell.
  /// @param y The y-coordinate of the cell.
  /// @return True if is edge of map and free cell.
  static bool explorableEdge(
    const nav_msgs::msg::OccupancyGrid & map_data,
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

  /// @brief Calculates the distance between a cell and coordinate.
  /// @param cell A map cell.
  /// @param coordinate A map coordinate position.
  /// @return The euclidean distance in meters.
  static double cellCoordDistance(
    const Cell & cell, const Coord coordinate,
    const nav_msgs::msg::OccupancyGrid & map_data);

  static Cell selectByDistance(const std::vector<Cell> & candidates, 
  const Coord robot_vp_position,
  const nav_msgs::msg::OccupancyGrid & map_data);

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
    const nav_msgs::msg::OccupancyGrid & map_data,
    int index,
    double rad);

  /// @brief Overloaded fcn scores a list of candidates by calculating the entropy of various state updates.
  /// @details This is an overloaded function that does NOT consider banned locations
  /// @param frontiers The list of frontier coordinates (x, y).
  /// @param map_data The occupancy grid map data.
  /// @param entropy_radius The radius around candidate cells to consider for the state update.
  /// @return The best scoring cell, the list of entropies, and the index of the best entropy.
  /// @see scoreByEntropy(const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data, double entropy_radius, BannedAreas banned);
  static std::tuple<Cell, std::vector<double>, int> scoreByEntropy(
    const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data,
    double entropy_radius);

  /// @brief Scores a list of candidates by calculating the entropy of various state updates.
  /// @details This is an overloaded function that does NOT consider banned locations
  /// @param frontiers The list of frontier coordinates (x, y).
  /// @param map_data The occupancy grid map data.
  /// @param entropy_radius The radius around candidate cells to consider for the state update.
  /// @param banned The struct of coordinates considered to be banned from goal pose consideration.
  /// @return The best scoring cell, the list of entropies, and the index of the best entropy.
  static std::tuple<Cell, std::vector<double>, int> scoreByEntropy(
    const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data,
    double entropy_radius,
    BannedAreas banned);

  /// @brief Overloaded fcn scores a list of candidates by calculating the unknown cells flipped in various state updates.
  /// @details This is an overloaded function that does NOT consider banned locations
  /// @param frontiers The list of frontier coordinates (x, y).
  /// @param map_data The occupancy grid map data.
  /// @param entropy_radius The radius around candidate cells to consider for the state update.
  /// @return The best scoring cell, the list of entropies, and the index of the best entropy.
  /// @see scoreByFlipCount(const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data, double entropy_radius, BannedAreas banned);
  static std::tuple<Cell, std::vector<int>, int> scoreByFlipCount(
    const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data,
    double entropy_radius);

  /// @brief Scores a list of candidates by calculating the unknown cells flipped in various state updates.
  /// @details This is an overloaded function that does NOT consider banned locations
  /// @param frontiers The list of frontier coordinates (x, y).
  /// @param map_data The occupancy grid map data.
  /// @param entropy_radius The radius around candidate cells to consider for the state update.
  /// @param banned The struct of coordinates considered to be banned from goal pose consideration.
  /// @return The best scoring cell, the list of entropies, and the index of the best entropy.
  static std::tuple<Cell, std::vector<int>, int> scoreByFlipCount(
    const std::vector<Cell> & frontiers, const nav_msgs::msg::OccupancyGrid & map_data,
    double entropy_radius,
    BannedAreas banned);

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
};

#endif // FRONTIER_HELPER_HPP
