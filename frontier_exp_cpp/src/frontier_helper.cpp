#include "frontier_exp_cpp/frontier_helper.hpp"

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

  std::vector<std::pair<int, int>> neighbors = {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
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
          !occluded(col, row, center_col, center_row, width, map_data.data))
        {
          unknown_count++;
        }
      }
    }
  }

  return unknown_count;
}

std::vector<int> FrontierHelper::performDBSCAN(
  const cv::Mat & points,
  float eps,
  int min_samples)
{
  std::vector<int> labels(points.rows, -1);    // Initialize labels to -1 (noise)
  int cluster_id = 0;

  // Helper to calculate neighbors
  auto find_neighbors = [&](int idx) {
      std::vector<int> neighbors;
      for (int i = 0; i < points.rows; ++i) {
        if (cv::norm(points.row(idx) - points.row(i)) <= eps) {
          neighbors.push_back(i);
        }
      }
      return neighbors;
    };

  // Core DBSCAN logic
  for (int i = 0; i < points.rows; ++i) {
    if (labels.at(i) != -1) {
      continue;                         // Skip already labeled points
    }
    auto neighbors = find_neighbors(i);
    if (static_cast<int>(neighbors.size()) < min_samples) {
      labels.at(i) = -1;        // Mark as noise
      continue;
    }

    // Start a new cluster
    labels.at(i) = cluster_id;
    std::vector<int> to_expand = neighbors;

    while (!to_expand.empty()) {
      int pt = to_expand.back();
      to_expand.pop_back();

      if (labels[pt] == -1) {
        labels[pt] = cluster_id;          // Change noise to border point
      }

      if (labels[pt] != -1) {
        continue;                            // Skip if already processed
      }

      // Check if the point is a valid core point
      auto new_neighbors = find_neighbors(pt);
      if (static_cast<int>(new_neighbors.size()) >= min_samples) {
        to_expand.insert(to_expand.end(), new_neighbors.begin(), new_neighbors.end());
      }

      // Assign to cluster even if it's a border point (but not a new core point)
      labels[pt] = cluster_id;
    }

    cluster_id++;
  }

  return labels;
}
