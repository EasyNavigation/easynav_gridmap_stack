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
/// \brief Declaration of the GridMapAStarPlanner class implementing A* path planning using elevation maps.

#ifndef EASYNAV_PLANNER__GRIDMAPASTARPLANNER_HPP_
#define EASYNAV_PLANNER__GRIDMAPASTARPLANNER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <expected>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

#include "grid_map_core/GridMap.hpp"

namespace easynav
{

/// \brief Planner based on A* algorithm using GridMap elevation data
class GridMapAStarPlanner : public PlannerMethodBase
{
public:
  /// \brief Default constructor
  explicit GridMapAStarPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /// \brief Computes a path using A* algorithm
  /// \param nav_state Current navigation state (with odometry and goals)
  void update(NavState & nav_state) override;

protected:
  double robot_radius_;
  double clearance_distance_;
  double max_allowed_slope_;

  nav_msgs::msg::Path current_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// \brief Internal A* implementation
  /// \param map The grid map with elevation layer
  /// \param start Pose in world coordinates
  /// \param goal Pose in world coordinates
  /// \return List of poses representing the path
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

  /// \brief Checks if the slope between two cells is below threshold
  /// \param map The grid map
  /// \param from Index of the first cell
  /// \param to Index of the second cell
  /// \return true if slope is acceptable, false otherwise
  bool isTraversable(
    const grid_map::GridMap & map,
    const grid_map::Index & from,
    const grid_map::Index & to) const;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__GRIDMAPASTARPLANNER_HPP_
