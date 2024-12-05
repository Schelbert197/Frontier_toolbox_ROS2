#include "frontier_exp_cpp/frontier_helper.hpp"
#include <random> // Required for random number generation in implementation
#include <algorithm> // Required for std::shuffle in implementation
#include <numeric> // Required for std::iota in implementation
#include <set>
#include <rclcpp/rclcpp.hpp> // For logger statements

std::vector<FrontierHelper::Cell> FrontierHelper::findFrontiers(
  const nav_msgs::msg::OccupancyGrid & map_data,
  bool consider_free_edge)
{
  std::vector<FrontierHelper::Cell> frontiers;

  // Loop through the map and find frontiers
  int height = map_data.info.height;
  int width = map_data.info.width;
  const auto & data = map_data.data;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * width + x;
      if (data[idx] == -1 &&
        FrontierHelper::hasFreeNeighbor(map_data, x, y))
      {
        frontiers.emplace_back(x, y);
      } else if (consider_free_edge && FrontierHelper::explorableEdge(
          map_data, x,
          y))
      {
        frontiers.emplace_back(x, y);
      }
    }
  }

  return frontiers;
}

std::map<FrontierHelper::Coord, int> FrontierHelper::stageBanned(
  const Cell & cell, std::map<Coord, int> & staged,
  double rad,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  // Convert to world coords, add if within radius, or add if not in list
  auto cell_world = cellToWorld(cell, map_data);

  // Check if the location is within the radius of any staged location
  for (const auto &[staged_loc, count] : staged) {
    double distance = std::hypot(
      cell_world.first - staged_loc.first,
      cell_world.second - staged_loc.second);
    if (distance < rad) {
      staged[staged_loc]++;
      return staged; // Exit early and don't add a new key
    }
  }

  // No close location was found, add a new entry
  staged[cell_world] = 1;
  return staged;
}

FrontierHelper::BannedAreas FrontierHelper::addBanned(
  std::map<Coord, int> & staged,
  FrontierHelper::BannedAreas & banned)
{
  // Check if any locations meet the threshold to be banned
  for (const auto &[staged_loc, count] : staged) {
    if (count == 3) {
      banned.coords.emplace_back(staged_loc); // Banish location
      staged[staged_loc]++; // Increment count again so we don't add loc to banned twice
      std::cout << "Location:" << staged_loc.first << ", " << staged_loc.second
                << " is now banished!" << std::endl; // Print newly banished location
    }
  }
  return banned;
}

bool FrontierHelper::identifyBanned(
  const Cell & cell, const BannedAreas & banned,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  // Convert to world coords, add if within radius, or add if not in list
  auto cell_world = cellToWorld(cell, map_data);

  for (const auto & loc : banned.coords) {
    double distance = std::hypot(
      cell_world.first - loc.first,
      cell_world.second - loc.second);
    if (distance < banned.radius) {
      return true;
    }
  }
  return false;
}

bool FrontierHelper::isPositionOutsideMap(
  const nav_msgs::msg::OccupancyGrid & map,
  const double & robot_x, const double & robot_y)
{
  double map_origin_x = map.info.origin.position.x;
  double map_origin_y = map.info.origin.position.y;
  double map_resolution = map.info.resolution;
  int map_width = map.info.width;
  int map_height = map.info.height;

  int grid_x = static_cast<int>((robot_x - map_origin_x) / map_resolution);
  int grid_y = static_cast<int>((robot_y - map_origin_y) / map_resolution);

  return grid_x < 0 || grid_x >= map_width || grid_y < 0 || grid_y >= map_height;
}

bool FrontierHelper::hasFreeNeighbor(
  const nav_msgs::msg::OccupancyGrid & map_data,
  int x, int y)
{
  int height = map_data.info.height;
  int width = map_data.info.width;
  const auto & data = map_data.data;

  std::vector<Cell> neighbors = {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
  for (const auto & [nx, ny] : neighbors) {
    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
      int idx = ny * width + nx;
      if (data[idx] == 0) {
        return true;
      }
    }
  }
  return false;
}

bool FrontierHelper::explorableEdge(
  const nav_msgs::msg::OccupancyGrid & map_data,
  int x, int y)
{
  // Extract map dimensions
  const int width = map_data.info.width;
  const int height = map_data.info.height;

  // Ensure (x, y) is within valid bounds
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return false; // Out of bounds, cannot be an edge
  }

  // Compute the index in the data array and check if cell is empty
  int index = y * width + x;
  if (map_data.data[index] != 0) {
    return false; // Not empty, so not considered an edge
  }

  // Check if the cell is on the edge of the map
  // A cell is on the edge if it's in the first/last row or first/last column
  return x == 0 || x == width - 1 || y == 0 || y == height - 1;
}

bool FrontierHelper::occluded(
  int x1, int y1, int x2, int y2, int width,
  const std::vector<int8_t> & map_data)
{
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  while (x1 != x2 || y1 != y2) {
    int cell_index = y1 * width + x1;
    if (map_data[cell_index] == 100) {
      return true;
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
  return false;
}

double FrontierHelper::cellCoordDistance(
  const Cell & cell, const Coord coordinate,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  auto [fx, fy] = FrontierHelper::cellToWorld(cell, map_data);
  auto [rx, ry] = coordinate;
  return std::hypot(fx - rx, fy - ry);
}

FrontierHelper::Cell FrontierHelper::selectByDistance(const std::vector<Cell> & candidates, 
  const Coord robot_vp_position,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  // Use std::min_element with a lambda function.
    auto goal_frontier = *std::min_element(
        candidates.begin(), candidates.end(),
        [&robot_vp_position, &map_data](const auto & f1, const auto & f2) {
            return FrontierHelper::cellCoordDistance(f1, robot_vp_position, map_data) < 
                   FrontierHelper::cellCoordDistance(f2, robot_vp_position, map_data);
        });

    return goal_frontier;
}

double FrontierHelper::calculateEntropy(int cell_value)
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

int FrontierHelper::countUnknownCellsWithinRadius(
  const nav_msgs::msg::OccupancyGrid & map_data,
  int index,
  double rad)
{
  int unknown_count = 0;

  // Get map metadata
  int width = static_cast<int>(map_data.info.width);
  int height = static_cast<int>(map_data.info.height);
  double resolution = map_data.info.resolution;

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
        if (map_data.data.at(cell_index) == -1 &&
          !FrontierHelper::occluded(col, row, center_col, center_row, width, map_data.data))
        {
          unknown_count++;
        }
      }
    }
  }

  return unknown_count;
}

std::tuple<FrontierHelper::Cell, std::vector<double>, int> FrontierHelper::scoreByEntropy(
  const std::vector<FrontierHelper::Cell> & frontiers,
  const nav_msgs::msg::OccupancyGrid & map_data,
  double entropy_radius)
{
  double total_entropy = FrontierHelper::calculateMapEntropy(map_data.data);

  // Establish objeccts
  std::vector<double> entropies;
  int best_frontier_idx;

  // Loop through all frontiers and get score
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto & frontier = frontiers.at(i);
    int idx = frontier.second * map_data.info.width + frontier.first;
    int unknowns = FrontierHelper::countUnknownCellsWithinRadius(map_data, idx, entropy_radius);

    // calculate current reduced entropy and place in list
    entropies.emplace_back(
      total_entropy - (unknowns * FrontierHelper::calculateEntropy(-1)) +
      (unknowns * FrontierHelper::calculateEntropy(0)));
  }

  // Find and return best entropy and index
  auto [index, entropy] = FrontierHelper::bestEntropyIndexScore(entropies);
  best_frontier_idx = index;

  return std::make_tuple(frontiers.at(best_frontier_idx), entropies, best_frontier_idx);
}

std::tuple<FrontierHelper::Cell, std::vector<double>, int> FrontierHelper::scoreByEntropy(
  const std::vector<FrontierHelper::Cell> & frontiers,
  const nav_msgs::msg::OccupancyGrid & map_data,
  double entropy_radius, FrontierHelper::BannedAreas banned)
{
  double total_entropy = FrontierHelper::calculateMapEntropy(map_data.data);

  // Establish objeccts
  std::vector<double> entropies;
  int best_frontier_idx;

  // Loop through all frontiers and get score
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto & frontier = frontiers.at(i);
    int idx = frontier.second * map_data.info.width + frontier.first;
    int unknowns = FrontierHelper::countUnknownCellsWithinRadius(map_data, idx, entropy_radius);
    if (FrontierHelper::identifyBanned(frontiers.at(i), banned, map_data)) {
      unknowns = 0;
    }

    // calculate current reduced entropy and place in list
    entropies.emplace_back(
      total_entropy - (unknowns * FrontierHelper::calculateEntropy(-1)) +
      (unknowns * FrontierHelper::calculateEntropy(0)));
  }

  // Find and return best entropy and index
  auto [index, entropy] = FrontierHelper::bestEntropyIndexScore(entropies);
  best_frontier_idx = index;

  return std::make_tuple(frontiers.at(best_frontier_idx), entropies, best_frontier_idx);
}

std::tuple<FrontierHelper::Cell, std::vector<int>, int> FrontierHelper::scoreByFlipCount(
  const std::vector<FrontierHelper::Cell> & frontiers,
  const nav_msgs::msg::OccupancyGrid & map_data,
  double entropy_radius)
{
  // Establish objeccts
  std::vector<int> unknowns_flipped;
  int best_frontier_idx;

  // Loop through all frontiers and get score
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto & frontier = frontiers.at(i);
    int idx = frontier.second * map_data.info.width + frontier.first;
    int unknowns = FrontierHelper::countUnknownCellsWithinRadius(map_data, idx, entropy_radius);

    unknowns_flipped.emplace_back(unknowns);
  }

  // Find and return best score and index
  auto [index, score] = FrontierHelper::bestUnknownsIndexScore(unknowns_flipped);
  best_frontier_idx = index;

  return std::make_tuple(frontiers.at(best_frontier_idx), unknowns_flipped, best_frontier_idx);
}

std::tuple<FrontierHelper::Cell, std::vector<int>, int> FrontierHelper::scoreByFlipCount(
  const std::vector<FrontierHelper::Cell> & frontiers,
  const nav_msgs::msg::OccupancyGrid & map_data,
  double entropy_radius,
  FrontierHelper::BannedAreas banned)
{
  // Establish objeccts
  std::vector<int> unknowns_flipped;
  int best_frontier_idx;

  // Loop through all frontiers and get score
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto & frontier = frontiers.at(i);
    int idx = frontier.second * map_data.info.width + frontier.first;
    int unknowns = FrontierHelper::countUnknownCellsWithinRadius(map_data, idx, entropy_radius);
    if (FrontierHelper::identifyBanned(frontiers.at(i), banned, map_data)) {
      unknowns = 0;
    }

    unknowns_flipped.emplace_back(unknowns);
  }

  // Find and return best score and index
  auto [index, score] = FrontierHelper::bestUnknownsIndexScore(unknowns_flipped);
  best_frontier_idx = index;

  return std::make_tuple(frontiers.at(best_frontier_idx), unknowns_flipped, best_frontier_idx);
}

std::pair<int, double> FrontierHelper::bestEntropyIndexScore(const std::vector<double> & entropies)
{
  // Select least entropy from list and find index
  auto min_iterator = std::min_element(entropies.begin(), entropies.end());
  double best_possible_entropy = *min_iterator;
  int best_frontier_idx_ = std::distance(entropies.begin(), min_iterator);

  return std::make_pair(best_frontier_idx_, best_possible_entropy);
}

FrontierHelper::Cell FrontierHelper::bestUnknownsIndexScore(const std::vector<int> & unknowns)
{
  // Select most converted unknowns from list and find index
  auto max_iterator = std::max_element(unknowns.begin(), unknowns.end());
  int best_possible_unknowns = *max_iterator;
  int best_frontier_idx_ = std::distance(unknowns.begin(), max_iterator);

  return std::make_pair(best_frontier_idx_, best_possible_unknowns);
}

double FrontierHelper::calculateMapEntropy(const std::vector<int8_t> & map_data)
{
  double entropy = 0.0;
  for (const auto & cell : map_data) {
    entropy += FrontierHelper::calculateEntropy(cell);  // Ensure this method is implemented
  }
  return entropy;
}

std::vector<FrontierHelper::Cell> FrontierHelper::sampleRandomFrontiers(
  const std::vector<FrontierHelper::Cell> & frontiers,
  size_t sample_size)
{
  std::vector<Cell> sampled_frontiers;

  // If the size of frontiers is less than or equal to the sample size, return the original frontiers
  if (frontiers.size() <= sample_size) {
    return frontiers;
  }

  // Create a list of indices from 0 to frontiers.size() - 1
  std::vector<size_t> indices(frontiers.size());
  std::iota(indices.begin(), indices.end(), 0);   // Fill indices with sequential numbers

  // Set up random engine and shuffle the indices
  std::random_device rd;
  std::mt19937 gen(rd());   // Mersenne Twister RNG
  std::shuffle(indices.begin(), indices.end(), gen);

  // Select the first 'sample_size' indices and fetch corresponding frontiers
  for (size_t i = 0; i < sample_size; ++i) {
    sampled_frontiers.push_back(frontiers[indices.at(i)]);
  }

  return sampled_frontiers;
}

std::pair<double, double> FrontierHelper::cellToWorld(
  const FrontierHelper::Cell & cell,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  double world_x = map_data.info.origin.position.x + (cell.first * map_data.info.resolution);
  double world_y = map_data.info.origin.position.y + (cell.second * map_data.info.resolution);
  return {world_x, world_y};
}

std::vector<FrontierHelper::Cell> FrontierHelper::getCentroidCells(
  const nav_msgs::msg::OccupancyGrid & map,
  std::vector<FrontierHelper::Coord> centroids)
{
  std::vector<Cell> cell_clusters;

  for (const auto & centroid : centroids) {
    int cell_x = (centroid.first - map.info.origin.position.x) / map.info.resolution;
    int cell_y = (centroid.second - map.info.origin.position.y) / map.info.resolution;
    cell_clusters.emplace_back(std::make_pair(cell_x, cell_y));
  }
  return cell_clusters;
}
