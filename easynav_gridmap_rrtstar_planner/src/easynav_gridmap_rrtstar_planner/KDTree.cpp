// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include "easynav_gridmap_rrtstar_planner/KDTree.hpp"
#include <cmath>
#include <algorithm>

namespace easynav
{

std::unique_ptr<KDTree::KDNode> KDTree::build(
  std::vector<std::shared_ptr<RRTNode>> & nodes,
  int depth)
{
  if (nodes.empty()) {return nullptr;}

    // Determine splitting axis (0 = x, 1 = y)
  int axis = depth % 2;

    // Sort nodes along the current axis
  std::sort(nodes.begin(), nodes.end(),
    [axis](const std::shared_ptr<RRTNode> & a, const std::shared_ptr<RRTNode> & b) {
      return axis == 0 ? a->index.x() < b->index.x() : a->index.y() < b->index.y();
              });

    // Choose median as root of this subtree
  int median = nodes.size() / 2;
  auto node = std::make_unique<KDNode>(nodes[median], depth);

    // Recursively build left and right subtrees
  std::vector<std::shared_ptr<RRTNode>> left_nodes(nodes.begin(), nodes.begin() + median);
  std::vector<std::shared_ptr<RRTNode>> right_nodes(nodes.begin() + median + 1, nodes.end());

  node->left = build(left_nodes, depth + 1);
  node->right = build(right_nodes, depth + 1);

  return node;
}

void KDTree::search_radius(
  const std::unique_ptr<KDNode> & node,
  const grid_map::Index & target,
  double radius_sq,
  std::vector<std::shared_ptr<RRTNode>> & result)
{
  if (!node) {return;}

    // Compute squared distance between node and target
  double dx = node->rrt_node->index.x() - target.x();
  double dy = node->rrt_node->index.y() - target.y();
  double dist_sq = dx * dx + dy * dy;

    // If within radius and not the target itself, add to result
  if (dist_sq <= radius_sq && !(node->rrt_node->index == target).all()) {
    result.push_back(node->rrt_node);
  }

    // Determine axis and distance along that axis
  int axis = node->depth % 2;
  double axis_dist = (axis == 0) ? dx : dy;

    // Recursively search child nodes based on axis distance
  if (axis_dist <= 0) {
    search_radius(node->left, target, radius_sq, result);
    if (axis_dist * axis_dist <= radius_sq) {
      search_radius(node->right, target, radius_sq, result);
    }
  } else {
    search_radius(node->right, target, radius_sq, result);
    if (axis_dist * axis_dist <= radius_sq) {
      search_radius(node->left, target, radius_sq, result);
    }
  }
}

void KDTree::rebuild(std::vector<std::shared_ptr<RRTNode>> & nodes)
{
  root = build(nodes, 0);
}

std::vector<std::shared_ptr<RRTNode>> KDTree::find_neighbors(
  const grid_map::Index & index,
  double radius)
{
  std::vector<std::shared_ptr<RRTNode>> result;
  search_radius(root, index, radius * radius, result);
  return result;
}

} // namespace easynav
