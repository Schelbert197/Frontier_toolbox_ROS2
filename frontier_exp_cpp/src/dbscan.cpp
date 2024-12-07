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

#include "frontier_exp_cpp/dbscan.hpp"

std::map<int, std::vector<DBSCAN::Cell>> DBSCAN::clusterFrontiers(
  const std::vector<Cell> & frontiers, float eps,
  int min_samples)
{
  cv::Mat points;
  for (const auto & f : frontiers) {
    points.push_back(cv::Vec2f(f.first, f.second));
  }

  auto labels = DBSCAN::performDBSCAN(points, eps, min_samples);

  // Organize clusters
  std::map<int, std::vector<Cell>> clusters;
  for (size_t i = 0; i < frontiers.size(); ++i) {
    int label = labels.at(i);
    if (label >= 0) {        // Ignore noise
      clusters[label].emplace_back(frontiers.at(i));
    }
  }

  // Filter out small clusters
  auto filtered_clusters = DBSCAN::filterClusters(clusters, min_samples);

  // Merge adjacent clusters
  filtered_clusters = DBSCAN::mergeAdjacentClusters(filtered_clusters);

  // RCLCPP_INFO(get_logger(), "Number of frontiers: %ld", frontiers.size());
  // // Print cluster information
  // for (const auto & [id, cluster] : filtered_clusters) {
  //   std::cout << "Cluster " << id << " has " << cluster.size() << " points\n";
  // }

  // RCLCPP_INFO(get_logger(), "Number of clusters: %ld", filtered_clusters.size());
  std::cout << "Number of frontiers: " << frontiers.size() << "\n";

// Print cluster information
  for (const auto & [id, cluster] : filtered_clusters) {
    std::cout << "Cluster " << id << " has " << cluster.size() << " points\n";
  }

  std::cout << "Number of clusters: " << filtered_clusters.size() << "\n";
  return filtered_clusters;
}

std::map<int, std::vector<DBSCAN::Cell>> DBSCAN::filterClusters(
  const std::map<int, std::vector<Cell>> & clusters,
  int min_samples)
{
  std::map<int, std::vector<Cell>> filtered_clusters;
  for (const auto & [id, cluster] : clusters) {
    if (static_cast<int>(cluster.size()) >= min_samples) {
      filtered_clusters[id] = cluster;
    }
  }
  return filtered_clusters;
}

std::vector<int> DBSCAN::performDBSCAN(
  const cv::Mat & points,
  double eps,
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
      continue;                 // Skip already labeled points
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
        labels[pt] = cluster_id;  // Change noise to border point
      }

      if (labels[pt] != -1) {
        continue;                // Skip if already processed
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

std::map<int, std::vector<DBSCAN::Cell>> DBSCAN::mergeAdjacentClusters(
  const std::map<int, std::vector<DBSCAN::Cell>> & clusters)
{
  // Step 1: Create a map of cluster IDs to sets of cell coordinates
  std::map<int, std::set<Cell>> cluster_cells;
  for (const auto &[cluster_id, cells] : clusters) {
    cluster_cells[cluster_id] = std::set<Cell>(cells.begin(), cells.end());
  }

  // Step 2: Track cluster merging using a union-find structure
  std::map<int, int> parent;   // Maps each cluster ID to its representative (parent)
  for (const auto &[cluster_id, _] : clusters) {
    parent[cluster_id] = cluster_id;     // Initially, each cluster is its own parent
  }

  // Helper to find the root of a cluster
  auto find = [&](int x) {
      while (x != parent[x]) {
        parent[x] = parent[parent[x]];     // Path compression
        x = parent[x];
      }
      return x;
    };

  // Helper to union two clusters
  auto unite = [&](int x, int y) {
      int root_x = find(x);
      int root_y = find(y);
      if (root_x != root_y) {
        parent[root_y] = root_x;     // Merge y into x
      }
    };

  // Step 3: Check adjacency and merge clusters
  std::vector<Cell> offsets = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto &[id1, cells1] : cluster_cells) {
    for (const auto &[id2, cells2] : cluster_cells) {
      if (id1 >= id2) {
        continue;                     // Avoid duplicate checks
      }
      for (const auto & cell : cells1) {
        for (const auto & offset : offsets) {
          Cell neighbor = {cell.first + offset.first, cell.second + offset.second};
          if (cells2.find(neighbor) != cells2.end()) {
            unite(id1, id2);
            break;
          }
        }
      }
    }
  }

  // Step 4: Collect merged clusters
  std::map<int, std::vector<Cell>> merged_clusters;
  for (const auto &[cluster_id, cells] : clusters) {
    int root = find(cluster_id);
    merged_clusters[root].insert(merged_clusters[root].end(), cells.begin(), cells.end());
  }

  // Step 5: Re-index clusters to ensure ordinal indices
  std::map<int, std::vector<Cell>> reindexed_clusters;
  int new_index = 0;
  for (const auto &[_, cluster_cells] : merged_clusters) {
    reindexed_clusters[new_index++] = cluster_cells;
  }

  return reindexed_clusters;
}

int DBSCAN::findLargestCluster(
  const ClusterObj & cluster_obj, const FrontierHelper::BannedAreas & banned,
  const nav_msgs::msg::OccupancyGrid & map_data)
{
  int largest_index = -1;   // Initialize to an invalid index
  size_t max_size = 0;      // To store the largest size

  // Iterate through all clusters
  for (const auto & [cluster_id, cluster_cells] : cluster_obj.clusters) {
    // Use cell centroids instead of the cluster directly
    if (FrontierHelper::identifyBanned(
        cluster_obj.cell_centroids.at(cluster_id), banned,
        map_data))
    {
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

int DBSCAN::findSecondLargestCluster(
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
