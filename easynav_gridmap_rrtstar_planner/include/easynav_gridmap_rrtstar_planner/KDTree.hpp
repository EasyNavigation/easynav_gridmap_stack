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

/// \file
/// \brief Declaration of the KDTree class for efficient nearest neighbor search of RRT nodes.
#ifndef EASYNAV_GRIDMAP_RRTSTAR_PLANNER__KDTREE_HPP_
#define EASYNAV_GRIDMAP_RRTSTAR_PLANNER__KDTREE_HPP_

#include <vector>
#include <memory>
#include <algorithm>
#include "grid_map_core/GridMap.hpp"
#include "easynav_gridmap_rrtstar_planner/GridMapRRTStarPlanner.hpp"
#include "easynav_gridmap_rrtstar_planner/RRTNode.hpp"

namespace easynav
{

struct RRTNode;

/**
 * @brief KD-tree for efficient nearest neighbor search of RRT nodes in a grid map.
 *
 * Supports incremental rebuilding and radius-based neighbor queries.
 */
class KDTree {
private:
/**
 * @brief Internal node of the KD-tree.
 */
  struct KDNode
  {
    std::shared_ptr<RRTNode> rrt_node;     ///< Associated RRT node
    std::unique_ptr<KDNode> left;           ///< Left subtree
    std::unique_ptr<KDNode> right;          ///< Right subtree
    int depth;                               ///< Depth of this node in the tree

  /**
   * @brief KDNode constructor
   * @param node Pointer to associated RRTNode
   * @param d Depth in the KD-tree
   */
    KDNode(std::shared_ptr<RRTNode> node, int d)
    : rrt_node(node), depth(d)
    {
    }
  };

  std::unique_ptr<KDNode> root;   ///< Root of the KD-tree

/**
 * @brief Recursively builds a balanced KD-tree from a list of nodes.
 * @param nodes List of RRT nodes to include in the tree
 * @param depth Current depth (used to determine split axis)
 * @return Unique pointer to the root KDNode of the subtree
 */
  std::unique_ptr<KDNode> build(std::vector<std::shared_ptr<RRTNode>> & nodes, int depth);

/**
 * @brief Recursively searches the KD-tree for nodes within a given squared radius.
 * @param node Current subtree node
 * @param target Index to search neighbors around
 * @param radius_sq Squared radius for neighbor search
 * @param result Vector to store found neighbor nodes
 */
  void search_radius(
    const std::unique_ptr<KDNode> & node, const grid_map::Index & target,
    double radius_sq, std::vector<std::shared_ptr<RRTNode>> & result);

public:
/**
 * @brief Rebuilds the KD-tree from a given set of RRT nodes.
 * @param nodes Vector of RRT nodes to include
 */
  void rebuild(std::vector<std::shared_ptr<RRTNode>> & nodes);

/**
 * @brief Finds all RRT nodes within a given radius of a target index.
 * @param index Target index for neighbor search
 * @param radius Search radius in map units
 * @return Vector of shared pointers to RRT nodes within the radius
 */
  std::vector<std::shared_ptr<RRTNode>> find_neighbors(
    const grid_map::Index & index,
    double radius);
};

} // namespace easynav

#endif  // EASYNAV_GRIDMAP_RRTSTAR_PLANNER__KDTREE_HPP_
