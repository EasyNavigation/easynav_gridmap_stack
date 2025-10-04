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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easynav
{

class KDTree;

/**
 * @class GridMapRRTStarPlanner
 * @brief RRT* path planner implementation using elevation grid maps.
 *
 * This planner generates collision-free and slope-constrained paths.
 * Features include:
 * - RRT* optimization
 * - KD-tree accelerated nearest neighbor search
 * - Path smoothing (Catmull-Rom and linear interpolation)
 * - Visualization markers for debugging and RViz display
 */
class GridMapRRTStarPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Constructor. Initializes KD-tree and sets default parameters.
   */
  GridMapRRTStarPlanner();

  /**
   * @brief Initializes the planner plugin.
   *
   * Declares parameters and sets up internal state.
   * @return std::expected<void, std::string> Empty if successful, error string otherwise.
   */
  std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Main planner update function.
   *
   * Computes a path from the robot to the goal, updates internal state,
   * and publishes visualization if enabled.
   *
   * @param nav_state Navigation state containing robot pose, map, and goals.
   */
  void update(NavState & nav_state) override;

protected:
  // ------------------------------------------------------------------
  // Core RRT* methods
  // ------------------------------------------------------------------

  /**
   * @brief Runs the RRT* algorithm to plan a path from start to goal.
   * @param map Grid map used for planning.
   * @param start Start pose in map frame.
   * @param goal Goal pose in map frame.
   * @return Vector of smoothed poses representing the planned path.
   *         Empty if start/goal invalid or no path found.
   */
  std::vector<geometry_msgs::msg::Pose> rrt_star(
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

  /**
   * @brief Generate a random grid index inside the map bounds.
   * @param map Grid map to sample from.
   * @param rng Random number generator.
   * @return Random index within map bounds.
   */
  grid_map::Index random_index(const grid_map::GridMap & map, std::mt19937 & rng);

  /**
   * @brief Generate a biased random index pointing forward from robot pose.
   * @param map Grid map.
   * @param robot_pose Current robot pose.
   * @param robot_yaw Robot orientation in radians.
   * @param rng Random generator.
   * @return Random index biased in forward direction.
   */
  grid_map::Index biased_random_index(
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & robot_pose,
    double robot_yaw,
    std::mt19937 & rng);

  /**
   * @brief Steer from one index toward another within max step size.
   * @param map Grid map.
   * @param from Starting index.
   * @param to Target index.
   * @return New index reached after stepping, or {-1,-1} if outside map.
   */
  grid_map::Index steer(
    const grid_map::GridMap & map,
    const grid_map::Index & from,
    const grid_map::Index & to);

  /**
   * @brief Compute traversal cost between two indices.
   *
   * Considers elevation changes and slope constraints.
   *
   * @param map Grid map.
   * @param from Source index.
   * @param to Destination index.
   * @return Traversal cost, or infinity if slope exceeds max.
   */
  double traversal_cost(
    const grid_map::GridMap & map,
    const grid_map::Index & from,
    const grid_map::Index & to);

  /**
   * @brief Compute Euclidean distance between two map indices.
   * @param map Grid map.
   * @param a First index.
   * @param b Second index.
   * @return Distance in meters.
   */
  double distance(
    const grid_map::GridMap & map,
    const grid_map::Index & a,
    const grid_map::Index & b);

  /**
   * @brief Generate a sample index for RRT* expansion.
   *
   * Sampling may be goal-biased, path-biased, or uniform random.
   *
   * @param map Grid map.
   * @param goal_idx Goal index.
   * @param best_goal_node Current best goal node.
   * @param rng Random generator.
   * @param prob_dist Probability distribution.
   * @return Sampled grid index.
   */
  grid_map::Index generate_sample(
    const grid_map::GridMap & map,
    const grid_map::Index & goal_idx,
    std::shared_ptr<RRTNode> best_goal_node,
    std::mt19937 & rng,
    std::uniform_real_distribution<double> & prob_dist);

  /**
   * @brief Find the nearest node in the tree to a target index.
   * @param tree RRT* tree of nodes.
   * @param target Target index.
   * @param map Grid map.
   * @return Shared pointer to nearest node, or nullptr if empty.
   */
  std::shared_ptr<RRTNode> find_nearest(
    const std::vector<std::shared_ptr<RRTNode>> & tree,
    const grid_map::Index & target,
    const grid_map::GridMap & map);

  /**
   * @brief Backtrack from goal node to extract the full path.
   * @param goal Goal node in the tree.
   * @param map Grid map.
   * @param goal_pose Final goal pose (orientation).
   * @return Vector of poses representing path.
   */
  std::vector<geometry_msgs::msg::Pose> extract_path(
    std::shared_ptr<RRTNode> goal,
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & goal_pose);

  // ------------------------------------------------------------------
  // Path management and smoothing
  // ------------------------------------------------------------------

  /**
   * @brief Merge new path poses with stored path and publish results.
   * @param new_poses Vector of new poses.
   * @param frame_id Map frame ID.
   */
  void combine_paths(
    const std::vector<geometry_msgs::msg::Pose> & new_poses,
    const std::string & frame_id);

  /**
   * @brief Generate a new path given robot and goal poses.
   * @param map Grid map.
   * @param robot_pose Current robot pose.
   * @param goal_pose Target goal pose.
   * @return Vector of poses forming a path.
   */
  std::vector<geometry_msgs::msg::Pose> generate_new_path(
    const grid_map::GridMap & map,
    const geometry_msgs::msg::Pose & robot_pose,
    const geometry_msgs::msg::Pose & goal_pose);

  /**
   * @brief Publish path visualization markers for RViz.
   * @param path Path message.
   */
  void publish_path_markers(const nav_msgs::msg::Path & path);

  /**
   * @brief Smooth path using Catmull-Rom splines.
   * @param input_path Original path.
   * @param interpolation_points_per_segment Number of interpolated points.
   * @return Smoothed path.
   */
  std::vector<geometry_msgs::msg::Pose> smooth_path(
    const std::vector<geometry_msgs::msg::Pose> & input_path,
    int interpolation_points_per_segment);

  /**
   * @brief Linearly interpolate between path points.
   * @param input_path Original path.
   * @param interpolation_points_per_segment Points per segment.
   * @return Interpolated path.
   */
  std::vector<geometry_msgs::msg::Pose> linear_interpolate_path(
    const std::vector<geometry_msgs::msg::Pose> & input_path,
    int interpolation_points_per_segment);

  /**
   * @brief Trim path to start from current robot pose.
   * @param path Original path.
   * @param robot_pose Current robot pose.
   * @return Trimmed path message.
   */
  nav_msgs::msg::Path trim_path_from_robot(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & robot_pose) const;

  /**
   * @brief Check if the goal has changed significantly since last update.
   * @param goal_pose New goal pose.
   * @return True if changed, false otherwise.
   */
  bool check_goal_changed(const geometry_msgs::msg::Pose & goal_pose);

  /**
   * @brief Compute lateral deviation of robot pose from a path.
   * @param path Path.
   * @param robot_pose Current pose.
   * @return Lateral deviation distance.
   */
  double calculate_lateral_distance_to_path(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & robot_pose) const;

  // ------------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------------
  double max_allowed_slope_deg_ = 50;              ///< Maximum slope in degrees
  double max_allowed_slope_ = 50.0 * M_PI / 180.0; ///< Maximum slope in radians
  int max_iters_ = 2000;                           ///< Maximum RRT* iterations
  double step_size_ = 0.8;                         ///< Step size in meters
  double neighbor_radius_ = 1.5;                   ///< Neighbor radius in meters
  double goal_bias_ = 0.1;                         ///< Probability of sampling goal
  double goal_threshold_ = 1.0;                    ///< Goal connection threshold (m)
  int kdtree_rebuild_interval_ = 200;              ///< KD-tree rebuild interval
  double spacing_ = 0.2;                           ///< Min spacing for smoothed path points
  double max_lateral_deviation_ = 1.5;             ///< Allowed lateral deviation (m)
  int final_poses_with_goal_orientation_ = 2;      ///< Preserve orientation for last poses

  // ------------------------------------------------------------------
  // Runtime state
  // ------------------------------------------------------------------
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;       ///< Publisher for path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; ///< Publisher for markers
  nav_msgs::msg::Path current_path_;                                ///< Current stored path
  std::unique_ptr<KDTree> kd_tree_;                                 ///< KD-tree for NN search
  std::unordered_map<uint64_t, double> cost_cache_;                 ///< Traversal cost cache
  uint32_t cached_map_width_ = 0;                                   ///< Cached map width
  uint32_t cached_map_height_ = 0;                                  ///< Cached map height
  uint64_t last_map_timestamp_ = 0;                                 ///< Cached map timestamp
  double last_path_cost_ = 0.0;                                     ///< Cost of previous path
  double current_path_cost_ = std::numeric_limits<double>::max();    ///< Cost of current path
  double new_path_cost_ = std::numeric_limits<double>::max();        ///< Cost of new candidate path
  int kdtree_rebuild_counter_ = 0;                                  ///< Counter for KD-tree rebuilds
  geometry_msgs::msg::Pose last_goal_pose_;                         ///< Last processed goal pose

  // ------------------------------------------------------------------
  // Cache utilities
  // ------------------------------------------------------------------

  /**
   * @brief Clear traversal cost cache.
   */
  void clear_cost_cache();

  /**
   * @brief Compute flat index from 2D index.
   * @param idx Grid index.
   * @param width Map width.
   * @return Flat index.
   */
  static uint32_t flat_index(const grid_map::Index & idx, uint32_t width);

  /**
   * @brief Generate unique key for an edge between two nodes.
   * @param a_flat Flat index of first node.
   * @param b_flat Flat index of second node.
   * @return Unique edge key.
   */
  static uint64_t edge_key(uint32_t a_flat, uint32_t b_flat);
};

} // namespace easynav

#endif // EASYNAV_GRIDMAP_RRTSTAR_PLANNER__GRIDMAPRRTSTARPLANNER_HPP_
