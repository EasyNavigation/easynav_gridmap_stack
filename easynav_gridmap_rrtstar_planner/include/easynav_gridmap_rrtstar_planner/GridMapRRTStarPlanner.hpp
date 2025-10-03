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
/// \brief Declaration of the GridMapRRTStarPlanner class implementing RRT* path planning using elevation maps.

#ifndef EASYNAV_GRIDMAP_RRTSTAR_PLANNER__GRIDMAPRRTSTARPLANNER_HPP_
#define EASYNAV_GRIDMAP_RRTSTAR_PLANNER__GRIDMAPRRTSTARPLANNER_HPP_

#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <random>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>

#include "grid_map_core/GridMap.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_gridmap_rrtstar_planner/KDTree.hpp"
#include "easynav_gridmap_rrtstar_planner/RRTNode.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace easynav
{

class KDTree;

    /**
     * @brief RRT* planner implementation using a grid map with elevation support.
     *
     * This planner generates collision-free paths that respect slope constraints.
     * It includes RRT* optimizations, KD-tree acceleration, and path smoothing.
     */
class GridMapRRTStarPlanner : public PlannerMethodBase
{
public:
        /**
         * @brief Constructor.
         * Initializes the KD-tree and sets default parameters.
         */
  GridMapRRTStarPlanner();

        /**
         * @brief Initializes the planner plugin.
         * Declares and retrieves parameters from the ROS2 node.
         * @return std::expected<void, std::string> Empty if successful, error message if failed.
         */
  std::expected<void, std::string> on_initialize() override;

        /**
         * @brief Main planner update function.
         * Computes a path from the robot to the goal each cycle.
         * Publishes the path and visualization markers if there are subscribers.
         * @param nav_state Navigation state containing robot pose, map, and goals.
         */
  void update(NavState & nav_state) override;

protected:
        /**
         * @brief Runs RRT* algorithm to generate a path from start to goal.
         * @param map Grid map used for planning.
         * @param start Start pose in the map frame.
         * @param goal Goal pose in the map frame.
         * @return Vector of smoothed poses representing the planned path.
         * @note Path will be empty if start or goal is outside map bounds.
         */
  std::vector<geometry_msgs::msg::Pose> rrt_star(
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

        /**
         * @brief Generates a random index inside the map.
         * @param map Grid map to sample from.
         * @param rng Random number generator.
         * @return Random index within map bounds.
         */
  grid_map::Index random_index(const grid_map::GridMap & map, std::mt19937 & rng);

        /**
         * @brief Steers from one index towards another with a maximum step size.
         * @param map Grid map used for distance conversion.
         * @param from Starting index.
         * @param to Target index.
         * @return New index reached after stepping, or {-1,-1} if outside map.
         */
  grid_map::Index steer(
    const grid_map::GridMap & map, const grid_map::Index & from,
    const grid_map::Index & to);

        /**
         * @brief Computes traversal cost between two indices, considering elevation and slope.
         * @param map Grid map used for cost evaluation.
         * @param to Destination index.
         * @param from Source index.
         * @return Traversal cost. Returns infinity if the path is invalid or slope exceeds max allowed.
         */
  double traversal_cost(
    const grid_map::GridMap & map, const grid_map::Index & to,
    const grid_map::Index & from);

        /**
         * @brief Computes Euclidean distance between two indices.
         * @param map Grid map used to convert indices to positions.
         * @param a First index.
         * @param b Second index.
         * @return Euclidean distance in meters.
         */
  double distance(
    const grid_map::GridMap & map, const grid_map::Index & a,
    const grid_map::Index & b);

        /**
         * @brief Extracts path poses from a goal node back to the start node.
         * @param node Goal node of RRT* tree.
         * @param map Grid map used for position extraction.
         * @param goal Goal pose for final orientation.
         * @return Vector of poses representing the path.
         */
  std::vector<geometry_msgs::msg::Pose> extract_path(
    std::shared_ptr<RRTNode> node,
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & goal);

        /**
         * @brief Finds the nearest node in the tree to a target index.
         * @param tree Vector of RRT nodes.
         * @param target Index to find nearest node to.
         * @param map Grid map used to compute distance.
         * @return Shared pointer to nearest node, or nullptr if tree is empty.
         */
  std::shared_ptr<RRTNode> find_nearest(
    const std::vector<std::shared_ptr<RRTNode>> & tree,
    const grid_map::Index & target,
    const grid_map::GridMap & map);

        /**
         * @brief Generates a biased sample index for RRT* exploration.
         * @param map Grid map used for sampling.
         * @param goal_idx Index of the goal.
         * @param best_goal_node Best goal node found so far (used for path-biased sampling).
         * @param rng Random number generator.
         * @param prob_dist Probability distribution for random sampling.
         * @return Index sampled for RRT* expansion.
         */
  grid_map::Index generate_sample(
    const grid_map::GridMap & map,
    const grid_map::Index & goal_idx,
    std::shared_ptr<RRTNode> best_goal_node,
    std::mt19937 & rng,
    std::uniform_real_distribution<double> & prob_dist);

        /**
         * @brief Smooths a path using Catmull-Rom splines.
         * @param input_path Input path to smooth.
         * @param interpolation_points_per_segment Number of interpolated points per segment.
         * @return Smoothed path as a vector of poses.
         * @note Spacing between consecutive points is enforced by `spacing_`.
         */
  std::vector<geometry_msgs::msg::Pose> smooth_path(
    const std::vector<geometry_msgs::msg::Pose> & input_path,
    int interpolation_points_per_segment);

        // -----------------------
        // Planner parameters
        // -----------------------
  double max_allowed_slope_deg_ = 50;                    ///< Maximum slope in degrees
  double max_allowed_slope_ = 50.0 * M_PI / 180.0;       ///< Maximum slope in radians
  int max_iters_ = 2000;                                 ///< Maximum RRT* iterations
  double step_size_ = 0.8;                               ///< Maximum step distance (m)
  double neighbor_radius_ = 1.5;                         ///< RRT* neighbor radius (m)
  double goal_bias_ = 0.1;                               ///< Probability of sampling the goal
  double goal_threshold_ = 1.0;                          ///< Distance threshold for goal connection (m)
  int kdtree_rebuild_interval_ = 200;                    ///< Interval to rebuild KD-tree
  double spacing_ = 0.2;                                 ///< Minimum spacing for smoothed path points (m)

        // -----------------------
        // Runtime state
        // -----------------------
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;       ///< Path publisher
  nav_msgs::msg::Path current_path_;                                 ///< Current planned path
  std::unique_ptr<KDTree> kd_tree_;                                  ///< KD-tree for nearest neighbor search
  std::unordered_map<uint64_t, double> cost_cache_;                  ///< Cache for traversal costs
  uint32_t cached_map_width_ = 0;
  uint32_t cached_map_height_ = 0;
  uint64_t last_map_timestamp_ = 0;

        // -----------------------
        // Cache utilities
        // -----------------------
  void clear_cost_cache();
  static uint32_t flat_index(const grid_map::Index & idx, uint32_t width);
  static uint64_t edge_key(uint32_t a_flat, uint32_t b_flat);
};

} // namespace easynav

#endif // EASYNAV_GRIDMAP_RRTSTAR_PLANNER__GRIDMAPRRTSTARPLANNER_HPP_
