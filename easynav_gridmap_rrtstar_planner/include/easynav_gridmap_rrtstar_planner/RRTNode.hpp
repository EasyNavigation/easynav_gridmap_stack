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
/// \brief Declaration of the RRTNode structure which represents a single node in the RRT* tree.

#ifndef EASYNAV_GRIDMAP_RRTSTAR_PLANNER__RRTNODE_HPP_
#define EASYNAV_GRIDMAP_RRTSTAR_PLANNER__RRTNODE_HPP_

#include <memory>
#include <vector>
#include "grid_map_core/GridMap.hpp"

namespace easynav
{

/**
 * @brief Represents a node in the RRT* tree.
 *
 * Stores the grid map index, parent and children pointers, and cumulative cost
 * from the root. Used by the KDTree and RRT* planner for path generation.
 */
struct RRTNode
{
  grid_map::Index index;                             ///< Grid cell index (x, y)
  std::shared_ptr<RRTNode> parent = nullptr;         ///< Pointer to parent node in the tree
  std::vector<std::shared_ptr<RRTNode>> children;    ///< Pointers to child nodes
  double cost = 0.0;                                 ///< Accumulated cost from root

    /// @brief Default constructor
  RRTNode() = default;

    /**
     * @brief Construct node with specific index and optional cost
     * @param idx Grid cell index
     * @param c Initial cost (default 0.0)
     */
  explicit RRTNode(const grid_map::Index & idx, double c = 0.0)
  : index(idx), cost(c)
  {
  }

    /**
     * @brief Change parent of this node and update cumulative cost
     * @param new_parent New parent node
     * @param new_cost Updated cumulative cost
     *
     * Note: children list is not automatically updated for performance reasons.
     */
  void change_parent(std::shared_ptr<RRTNode> new_parent, double new_cost)
  {
    parent = new_parent;
    cost = new_cost;
  }
};

} // namespace easynav

#endif // EASYNAV_GRIDMAP_RRTSTAR_PLANNER__RRTNODE_HPP_
