#include "frontier_exp_cpp/dbscan.hpp"

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