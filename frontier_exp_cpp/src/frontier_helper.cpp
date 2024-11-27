#include "frontier_exp_cpp/frontier_helper.hpp"
#include <random> // Required for random number generation in implementation
#include <algorithm> // Required for std::shuffle in implementation
#include <numeric> // Required for std::iota in implementation
#include <set>
#include <rclcpp/rclcpp.hpp> // For logger statements

  void findFrontiers(
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
          FrontierHelper::hasFreeNeighbor(map_data, x, y) && !tooClose(std::make_pair(x, y)))
        {
          frontiers.emplace_back(x, y);
        } else if (consider_free_edge && FrontierHelper::explorableEdge(
            map_data, x,
            y) && !tooClose(std::make_pair(x, y)))
        {
          frontiers.emplace_back(x, y);
        }
      }
    }
    // // Convert the vector to a string
    // std::stringstream ss;
    // ss << "Vector contents: ";
    // for (const auto & pair : frontiers) {
    //   ss << "(" << pair.first << ", " << pair.second << ") ";
    // }
    // // Print the vector contents of the frontier vector
    // RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
    
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
  nav_msgs::msg::OccupancyGrid & map_data,
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
  return (x == 0 || x == width - 1 || y == 0 || y == height - 1);
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
  nav_msgs::msg::OccupancyGrid & map_data,
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

// std::vector<int> FrontierHelper::performDBSCAN(
//   const cv::Mat & points,
//   double eps,
//   int min_samples)
// {
//   std::vector<int> labels(points.rows, -1);    // Initialize labels to -1 (noise)
//   int cluster_id = 0;

//   // Helper to calculate neighbors
//   auto find_neighbors = [&](int idx) {
//       std::vector<int> neighbors;
//       for (int i = 0; i < points.rows; ++i) {
//         if (cv::norm(points.row(idx) - points.row(i)) <= eps) {
//           neighbors.push_back(i);
//         }
//       }
//       return neighbors;
//     };

//   // Core DBSCAN logic
//   for (int i = 0; i < points.rows; ++i) {
//     if (labels.at(i) != -1) {
//       continue;                 // Skip already labeled points
//     }
//     auto neighbors = find_neighbors(i);
//     if (static_cast<int>(neighbors.size()) < min_samples) {
//       labels.at(i) = -1;        // Mark as noise
//       continue;
//     }

//     // Start a new cluster
//     labels.at(i) = cluster_id;
//     std::vector<int> to_expand = neighbors;

//     while (!to_expand.empty()) {
//       int pt = to_expand.back();
//       to_expand.pop_back();

//       if (labels[pt] == -1) {
//         labels[pt] = cluster_id;  // Change noise to border point
//       }

//       if (labels[pt] != -1) {
//         continue;                // Skip if already processed
//       }

//       // Check if the point is a valid core point
//       auto new_neighbors = find_neighbors(pt);
//       if (static_cast<int>(new_neighbors.size()) >= min_samples) {
//         to_expand.insert(to_expand.end(), new_neighbors.begin(), new_neighbors.end());
//       }

//       // Assign to cluster even if it's a border point (but not a new core point)
//       labels[pt] = cluster_id;
//     }

//     cluster_id++;
//   }

//   return labels;
// }

// std::map<int, std::vector<FrontierHelper::Cell>> FrontierHelper::mergeAdjacentClusters(
//   const std::map<int, std::vector<FrontierHelper::Cell>> & clusters)
// {
//   // Step 1: Create a map of cluster IDs to sets of cell coordinates
//   std::map<int, std::set<Cell>> cluster_cells;
//   for (const auto &[cluster_id, cells] : clusters) {
//     cluster_cells[cluster_id] = std::set<Cell>(cells.begin(), cells.end());
//   }

//   // Step 2: Track cluster merging using a union-find structure
//   std::map<int, int> parent;   // Maps each cluster ID to its representative (parent)
//   for (const auto &[cluster_id, _] : clusters) {
//     parent[cluster_id] = cluster_id;     // Initially, each cluster is its own parent
//   }

//   // Helper to find the root of a cluster
//   auto find = [&](int x) {
//       while (x != parent[x]) {
//         parent[x] = parent[parent[x]];     // Path compression
//         x = parent[x];
//       }
//       return x;
//     };

//   // Helper to union two clusters
//   auto unite = [&](int x, int y) {
//       int root_x = find(x);
//       int root_y = find(y);
//       if (root_x != root_y) {
//         parent[root_y] = root_x;     // Merge y into x
//       }
//     };

//   // Step 3: Check adjacency and merge clusters
//   std::vector<Cell> offsets = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
//   for (const auto &[id1, cells1] : cluster_cells) {
//     for (const auto &[id2, cells2] : cluster_cells) {
//       if (id1 >= id2) {
//         continue;                     // Avoid duplicate checks
//       }
//       for (const auto & cell : cells1) {
//         for (const auto & offset : offsets) {
//           Cell neighbor = {cell.first + offset.first, cell.second + offset.second};
//           if (cells2.find(neighbor) != cells2.end()) {
//             unite(id1, id2);
//             break;
//           }
//         }
//       }
//     }
//   }

//   // Step 4: Collect merged clusters
//   std::map<int, std::vector<Cell>> merged_clusters;
//   for (const auto &[cluster_id, cells] : clusters) {
//     int root = find(cluster_id);
//     merged_clusters[root].insert(merged_clusters[root].end(), cells.begin(), cells.end());
//   }

//   // Step 5: Re-index clusters to ensure ordinal indices
//   std::map<int, std::vector<Cell>> reindexed_clusters;
//   int new_index = 0;
//   for (const auto &[_, cluster_cells] : merged_clusters) {
//     reindexed_clusters[new_index++] = cluster_cells;
//   }

//   return reindexed_clusters;
// }

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

int FrontierHelper::findLargestCluster(
  const ClusterObj & cluster_obj, const BannedAreas & banned,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  int largest_index = -1;   // Initialize to an invalid index
  size_t max_size = 0;      // To store the largest size

  // Iterate through all clusters
  for (const auto & [cluster_id, cluster_cells] : cluster_obj.clusters) {
    // Use cell centroids instead of the cluster directly
    if (FrontierHelper::identifyBanned(cluster_obj.cell_centroids.at(cluster_id), banned, map_data)) {
      continue;
    }

    // Check if the current cluster is larger than the max found so far
    if (cluster_cells.size() > max_size) {
      max_size = cluster_cells.size();
      largest_index = cluster_id;  // Update to the current cluster's ID
    }
  }

  return largest_index;   // Return the index of the largest cluster
}

int FrontierHelper::findSecondLargestCluster(
  const std::map<int, std::vector<FrontierHelper::Cell>> & clusters)
{
  int largest_index = -1;         // To store the index of the largest cluster
  int second_largest_index = -1;   // To store the index of the second largest cluster
  size_t max_size = 0;             // Size of the largest cluster
  size_t second_max_size = 0;      // Size of the second largest cluster

  for (const auto & cluster : clusters) {
    size_t cluster_size = cluster.second.size();

    if (cluster_size > max_size) {
      // Update second largest to current largest
      second_max_size = max_size;
      second_largest_index = largest_index;

      // Update largest
      max_size = cluster_size;
      largest_index = cluster.first;
    } else if (cluster_size > second_max_size) {
      // Update second largest
      second_max_size = cluster_size;
      second_largest_index = cluster.first;
    }
  }

  return second_largest_index;   // Return the index of the second largest cluster
}
