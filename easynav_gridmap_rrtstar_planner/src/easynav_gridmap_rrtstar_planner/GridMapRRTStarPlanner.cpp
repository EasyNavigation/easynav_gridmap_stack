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

#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <random>
#include <queue>
#include <unordered_set>

#include "grid_map_core/GridMap.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "easynav_gridmap_rrtstar_planner/GridMapRRTStarPlanner.hpp"
#include "easynav_gridmap_rrtstar_planner/KDTree.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easynav
{
grid_map::Index GridMapRRTStarPlanner::random_index(
  const grid_map::GridMap & map,
  std::mt19937 & rng)
{
  std::uniform_int_distribution<int> dist_x(0, map.getSize()[0] - 1);
  std::uniform_int_distribution<int> dist_y(0, map.getSize()[1] - 1);
  return {dist_x(rng), dist_y(rng)};
}

double compute_path_length(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  return total_length;
}

GridMapRRTStarPlanner::GridMapRRTStarPlanner()
: kd_tree_(std::make_unique<KDTree>())
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path)
    {
      std::ostringstream ret;
      ret     << "Path with " << path.poses.size() << " poses, length "
              << compute_path_length(path) << " m.";
      return ret.str();
        });
}

std::expected<void, std::string> GridMapRRTStarPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".max_allowed_slope_deg", 40.0);
  node->declare_parameter<int>(plugin_name + ".max_iters", 3000);
  node->declare_parameter<double>(plugin_name + ".step_size", 1.0);
  node->declare_parameter<double>(plugin_name + ".neighbor_radius", 2.0);
  node->declare_parameter<double>(plugin_name + ".goal_bias", 0.15);
  node->declare_parameter<double>(plugin_name + ".goal_threshold", 0.5);
  node->declare_parameter<int>(plugin_name + ".kdtree_rebuild_interval", 800);
  node->declare_parameter<double>(plugin_name + ".spacing", 0.2);
  node->declare_parameter<double>(plugin_name + ".max_lateral_deviation", 0.5);
  node->declare_parameter<int>(plugin_name + ".final_poses_with_goal_orientation", 2);

  node->get_parameter(plugin_name + ".max_allowed_slope_deg", max_allowed_slope_deg_);
  node->get_parameter(plugin_name + ".max_iters", max_iters_);
  node->get_parameter(plugin_name + ".step_size", step_size_);
  node->get_parameter(plugin_name + ".neighbor_radius", neighbor_radius_);
  node->get_parameter(plugin_name + ".goal_bias", goal_bias_);
  node->get_parameter(plugin_name + ".goal_threshold", goal_threshold_);
  node->get_parameter(plugin_name + ".kdtree_rebuild_interval", kdtree_rebuild_interval_);
  node->get_parameter(plugin_name + ".spacing", spacing_);
  node->get_parameter(plugin_name + ".max_lateral_deviation", max_lateral_deviation_);
  node->get_parameter(plugin_name + ".final_poses_with_goal_orientation",
      final_poses_with_goal_orientation_);

  max_allowed_slope_ = max_allowed_slope_deg_ * M_PI / 180.0;

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planner/path", 10);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("planner/marker", 10);

  node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  last_goal_pose_.position.x = 0.0;
  last_goal_pose_.position.y = 0.0;
  last_goal_pose_.position.z = 0.0;
  last_goal_pose_.orientation.x = 0.0;
  last_goal_pose_.orientation.y = 0.0;
  last_goal_pose_.orientation.z = 0.0;
  last_goal_pose_.orientation.w = 1.0;

  return {};
}

grid_map::Index GridMapRRTStarPlanner::steer(
  const grid_map::GridMap & map,
  const grid_map::Index & from,
  const grid_map::Index & to)
{

    // Convert indices to continuous map positions
  grid_map::Position p_from, p_to;
  map.getPosition(from, p_from);
  map.getPosition(to, p_to);

    // Compute vector from 'from' to 'to' and its Euclidean distance
  double dx = p_to.x() - p_from.x();
  double dy = p_to.y() - p_from.y();
  double dist = std::hypot(dx, dy);

  grid_map::Position p_new;
  if (dist <= step_size_) {
      // Target is within one step: move directly to it
    p_new = p_to;
  } else {
      // Move along the line by 'step_size_' toward target
    double ratio = step_size_ / dist;
    p_new = grid_map::Position(p_from.x() + dx * ratio,
                                 p_from.y() + dy * ratio);
  }

    // Convert continuous position back to map index
  grid_map::Index new_idx;
  if (!map.getIndex(p_new, new_idx)) {
      // New position is outside map bounds
    return grid_map::Index(-1, -1);
  }

  return new_idx;
}

std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::rrt_star(
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  std::vector<std::shared_ptr<RRTNode>> tree;
  static uint32_t seed_counter = 0;
  std::mt19937 rng(seed_counter++);
  std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

  grid_map::Index start_idx, goal_idx;
  if (!map.getIndex(grid_map::Position(start.position.x, start.position.y), start_idx) ||
    !map.getIndex(grid_map::Position(goal.position.x, goal.position.y), goal_idx))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Start or goal position is outside map bounds");
    return {};
  }

    // get robot yaw from the start pose for forward-biased sampling
  tf2::Quaternion robot_q;
  tf2::fromMsg(start.orientation, robot_q);
  double robot_roll, robot_pitch, robot_yaw;
  tf2::Matrix3x3(robot_q).getRPY(robot_roll, robot_pitch, robot_yaw);

  auto root = std::make_shared<RRTNode>();
  root->index = start_idx;
  root->cost = 0.0;
  tree.push_back(root);

  std::shared_ptr<RRTNode> best_goal_node = nullptr;
  double best_goal_cost = std::numeric_limits<double>::max();

  int iterations_without_improvement = 0;
  const int max_no_improvement = 500;
  const int min_iterations = max_iters_ / 2;

  for (int iter = 0; iter < max_iters_; ++iter) {
        // adaptive goal bias: increase bias if a candidate path exists, else grow modestly
    double adaptive_goal_bias = goal_bias_;

    if (best_goal_node) {
      adaptive_goal_bias = std::min(0.9, goal_bias_ + 0.15 * (iter / 100.0));
    } else {
      adaptive_goal_bias = std::min(0.6, goal_bias_ * 2.0);
    }

    grid_map::Index rand_idx;
    if (prob_dist(rng) < adaptive_goal_bias) {
      rand_idx = goal_idx;
    } else {
            // early iterations: bias samples forward relative to robot heading, otherwise sample uniformly
      if (iter < 100 && tree.size() < 50) {
        rand_idx = biased_random_index(map, start, robot_yaw, rng);
      } else {
        rand_idx = random_index(map, rng);
      }
    }

    auto nearest = find_nearest(tree, rand_idx, map);
    if (!nearest) {
      continue;
    }

    auto new_idx = steer(map, nearest->index, rand_idx);
    if ((new_idx == grid_map::Index(-1, -1)).all()) {
      continue;
    }

        // skip exact duplicates and very close nodes when tree is large
    bool duplicate = false;
    for (const auto & node : tree) {
      if ((node->index == new_idx).all()) {
        duplicate = true;
        break;
      }
      if (tree.size() > 500 && distance(map, node->index, new_idx) < 0.1) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {continue;}

        // compute full traversal cost for the new edge and skip infeasible edges
    double edge_cost = traversal_cost(map, nearest->index, new_idx);
    if (!std::isfinite(edge_cost)) {
      continue;
    }

    auto new_node = std::make_shared<RRTNode>();
    new_node->index = new_idx;
    new_node->cost = nearest->cost + edge_cost;
    new_node->parent = nearest;

        // collect nearby nodes for potential rewiring using a dynamic radius
    std::vector<std::shared_ptr<RRTNode>> nearby_nodes;
    double search_radius = std::min(neighbor_radius_,
                                       neighbor_radius_ *
        std::sqrt(std::log(tree.size()) / tree.size()));

    for (const auto & near_node : tree) {
      double dist = distance(map, near_node->index, new_idx);
      if (dist <= search_radius) {
        nearby_nodes.push_back(near_node);
      }
    }

        // limit number of neighbors considered, keeping closest ones
    if (nearby_nodes.size() > 8) {
      std::nth_element(nearby_nodes.begin(), nearby_nodes.begin() + 15, nearby_nodes.end(),
        [&](const auto & a, const auto & b) {
          return distance(map, a->index, new_idx) < distance(map, b->index, new_idx);
                           });
      nearby_nodes.resize(8);
    }

        // choose the best parent among nearby nodes to minimize cost
    for (const auto & near_node : nearby_nodes) {
      double cost = traversal_cost(map, near_node->index, new_idx);
      if (!std::isfinite(cost)) {
        continue;
      }

      double total = near_node->cost + cost;
      if (total < new_node->cost) {
        new_node->cost = total;
        new_node->parent = near_node;
      }
    }

    tree.push_back(new_node);

        // try to rewire nearby nodes through the new node if it lowers their cost
    for (auto & near_node : nearby_nodes) {
      if (near_node == new_node || near_node == root) {
        continue;
      }

      double cost = traversal_cost(map, new_idx, near_node->index);
      if (!std::isfinite(cost)) {
        continue;
      }

      double total = new_node->cost + cost;
      if (total < near_node->cost - 0.01) {        // small threshold to avoid oscillation
        near_node->cost = total;
        near_node->parent = new_node;
      }
    }

        // check proximity to goal and update best solution if improved
    double dist_to_goal = distance(map, new_idx, goal_idx);
    if (dist_to_goal < goal_threshold_) {
      double goal_cost = traversal_cost(map, new_idx, goal_idx);
      if (std::isfinite(goal_cost)) {
        double total = new_node->cost + goal_cost;
        if (total < best_goal_cost) {
          best_goal_node = new_node;
          best_goal_cost = total;
          iterations_without_improvement = 0;

          RCLCPP_DEBUG(get_node()->get_logger(),
                               "New best goal found at iteration %d with cost %.2f (dist: %.2f)",
                               iter, best_goal_cost, dist_to_goal);

                    // adaptive early termination: evaluate path efficiency against direct distance
          if (iter > min_iterations && best_goal_node) {
            double direct_distance = distance(map, start_idx, goal_idx);
            double path_efficiency = best_goal_cost / direct_distance;

            if (path_efficiency < 1.3 && iter > 2000) {
              RCLCPP_DEBUG(get_node()->get_logger(),
                                       "Early termination: efficient path found (%.1f%% of direct) at iter %d",
                                       path_efficiency * 100, iter);
              break;
            }

            if (path_efficiency < 1.1 && iter > 1000) {
              RCLCPP_DEBUG(get_node()->get_logger(),
                                       "Very efficient path found (%.1f%%), early exit at iter %d",
                                       path_efficiency * 100, iter);
              break;
            }
          }
        } else {
          iterations_without_improvement++;
        }
      }
    } else {
      iterations_without_improvement++;
    }

        // adaptive termination when no improvement for many iterations, scaled by tree size
    int max_no_improvement_adaptive = max_no_improvement;
    if (tree.size() > 1000) {max_no_improvement_adaptive = 50;}
    if (tree.size() > 2000) {max_no_improvement_adaptive = 30;}

    if (iterations_without_improvement > max_no_improvement_adaptive && best_goal_node) {
      RCLCPP_DEBUG(get_node()->get_logger(),
                       "No improvement termination at iteration %d (tree size: %zu)",
                       iter, tree.size());
      break;
    }
  }

  if (!best_goal_node) {
    RCLCPP_WARN(get_node()->get_logger(), "No path found to goal after %d iterations", max_iters_);
    return {};
  }

  auto raw_path = extract_path(best_goal_node, map, goal);

  if (raw_path.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Extracted path is empty");
    return {};
  }

    // apply conservative smoothing to the raw path
  raw_path = smooth_path(raw_path, 4);

  RCLCPP_DEBUG(get_node()->get_logger(),
                 "Generated RRT* path with %zu points, cost %.2f, tree size: %zu",
                 raw_path.size(), best_goal_cost, tree.size());

  new_path_cost_ = best_goal_cost;
  return raw_path;
}


grid_map::Index GridMapRRTStarPlanner::biased_random_index(
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & robot_pose,
  double robot_yaw,
  std::mt19937 & rng)
{
    // Generate samples biased forward relative to the robot heading.
    // Angle in ±60° in front, distance between 1 and 5 meters.
  std::uniform_real_distribution<double> angle_dist(-M_PI / 3, M_PI / 3);
  std::uniform_real_distribution<double> dist_dist(1.0, 5.0);

  double bias_angle = robot_yaw + angle_dist(rng);
  double bias_distance = dist_dist(rng);

    // Compute biased target position in world coordinates
  double target_x = robot_pose.position.x + bias_distance * std::cos(bias_angle);
  double target_y = robot_pose.position.y + bias_distance * std::sin(bias_angle);

  grid_map::Index biased_idx;
  grid_map::Position target_pos(target_x, target_y);

    // If biased point is inside the map, return its index, otherwise fall back
  if (map.getIndex(target_pos, biased_idx)) {
    return biased_idx;
  } else {
    return random_index(map, rng);
  }
}

std::shared_ptr<RRTNode> GridMapRRTStarPlanner::find_nearest(
  const std::vector<std::shared_ptr<RRTNode>> & tree,
  const grid_map::Index & target,
  const grid_map::GridMap & map)
{
  if (tree.empty()) {
    return nullptr;
  }

    // Return the node in 'tree' with minimum Euclidean distance to 'target'
  return *std::min_element(tree.begin(), tree.end(),
           [&](const auto & a, const auto & b) {
             return distance(map, a->index, target) < distance(map, b->index, target);
                             });
}

double GridMapRRTStarPlanner::distance(
  const grid_map::GridMap & map,
  const grid_map::Index & a,
  const grid_map::Index & b)
{
  grid_map::Position pa, pb;
  map.getPosition(a, pa);
  map.getPosition(b, pb);

  return std::hypot(pb.x() - pa.x(), pb.y() - pa.y());
}

// Replace previous implementation: trim path by removing waypoints behind or already passed by the robot
nav_msgs::msg::Path GridMapRRTStarPlanner::trim_path_from_robot(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & robot_pose) const
{
  nav_msgs::msg::Path trimmed_path = path;

  if (path.poses.size() < 2) {
    return trimmed_path;
  }

  const double trim_distance = 1.0;   // meters: threshold to consider a waypoint "passed"
  size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::max();

    // Find the closest waypoint to the robot
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & wp = path.poses[i].pose.position;
    const auto & rp = robot_pose.position;
    double d = std::hypot(wp.x - rp.x, wp.y - rp.y);
    if (d < min_distance) {
      min_distance = d;
      closest_index = i;
    }
  }

    // Identify previous waypoints that are very close (already visited)
  size_t start_index = closest_index;
  for (size_t i = 0; i <= closest_index; ++i) {
    const auto & wp = path.poses[i].pose.position;
    const auto & rp = robot_pose.position;
    double d = std::hypot(wp.x - rp.x, wp.y - rp.y);
    if (d < trim_distance) {
      start_index = i + 1;       // mark for removal
    } else {
      break;
    }
  }

    // Ensure we keep at least the closest waypoint
  if (start_index > closest_index) {
    start_index = closest_index;
  }

    // Erase the earlier segment only if enough waypoints remain after trimming
  if (start_index > 0 && (path.poses.size() - start_index) >= 3) {
    trimmed_path.poses.erase(trimmed_path.poses.begin(), trimmed_path.poses.begin() + start_index);
    RCLCPP_DEBUG(get_node()->get_logger(),
                     "Path trimmed: removed %zu waypoints, %zu remaining (closest at %.2fm)",
                     start_index, trimmed_path.poses.size(), min_distance);
  }

  return trimmed_path;
}

std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::extract_path(
  std::shared_ptr<RRTNode> goal_node,
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & goal)
{
  std::vector<std::shared_ptr<RRTNode>> nodes;
  nodes.reserve(100);

    // collect nodes from the RRT tree back to the root
  for (auto current = goal_node; current; current = current->parent) {
    nodes.push_back(current);
  }
  std::reverse(nodes.begin(), nodes.end());

  std::vector<geometry_msgs::msg::Pose> path;
  path.reserve(nodes.size());

    // convert tree nodes to basic poses (no smoothing here)
  for (size_t i = 0; i < nodes.size(); ++i) {
    const auto & node = nodes[i];
    grid_map::Position pos;
    map.getPosition(node->index, pos);

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = map.at("elevation", node->index);

        // DEFAULT ORIENTATION: neutral (facing along x-axis)
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.orientation = tf2::toMsg(q);

    path.push_back(pose);
  }

    // for the final raw waypoint, apply the goal orientation
  if (!path.empty()) {
    path.back().orientation = goal.orientation;
  }

  RCLCPP_DEBUG(get_node()->get_logger(),
                 "Path extracted with %zu raw RRT* nodes (no smoothing)", path.size());

  return path;
}

double GridMapRRTStarPlanner::calculate_lateral_distance_to_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & robot_pose) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::max();

    // Find the closest point on any path segment to the robot
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const auto & p1 = path.poses[i].pose.position;
    const auto & p2 = path.poses[i + 1].pose.position;
    const auto & rp = robot_pose.position;

        // Project robot position onto the vector p1->p2 and clamp to the segment
    double vx = p2.x - p1.x;
    double vy = p2.y - p1.y;
    double wx = rp.x - p1.x;
    double wy = rp.y - p1.y;

    double dot = wx * vx + wy * vy;
    double len_sq = vx * vx + vy * vy;

    if (len_sq < 1e-6) {
            // degenerate segment, skip
      continue;
    }

    double t = dot / len_sq;
    double cx, cy;
    if (t <= 0.0) {
      cx = p1.x;
      cy = p1.y;
    } else if (t >= 1.0) {
      cx = p2.x;
      cy = p2.y;
    } else {
      cx = p1.x + t * vx;
      cy = p1.y + t * vy;
    }

    double d = std::hypot(rp.x - cx, rp.y - cy);
    min_distance = std::min(min_distance, d);
  }

  return (min_distance == std::numeric_limits<double>::max()) ? 0.0 : min_distance;
}

double GridMapRRTStarPlanner::traversal_cost(
  const grid_map::GridMap & map,
  const grid_map::Index & from,
  const grid_map::Index & to)
{
    // Use a unique key (based on flattened indices) to identify the edge.
  const uint32_t width = static_cast<uint32_t>(map.getSize()[0]);
  const uint64_t key = edge_key(flat_index(from, width), flat_index(to, width));

  if (auto it = cost_cache_.find(key); it != cost_cache_.end()) {
    return it->second;
  }

  grid_map::Position pos_from, pos_to;
  map.getPosition(from, pos_from);
  map.getPosition(to, pos_to);

  if (!map.isInside(pos_from) || !map.isInside(pos_to)) {
    return std::numeric_limits<double>::infinity();
  }

  if (!map.isValid(from, "elevation") || !map.isValid(to, "elevation")) {
    return std::numeric_limits<double>::infinity();
  }

  double z1 = map.at("elevation", from);
  double z2 = map.at("elevation", to);
  double dx = map.getResolution() * (to.x() - from.x());
  double dy = map.getResolution() * (to.y() - from.y());
  double dz = z2 - z1;
  double horizontal_dist = std::hypot(dx, dy);

    // Flat surface: cost = horizontal distance
  if (std::abs(dz) < 1e-3) {
    return cost_cache_[key] = horizontal_dist;
  }

    // Calculate slope
  double slope_rad = std::atan2(std::abs(dz), horizontal_dist);

    // Reject edges with slope exceeding maximum allowed
  if (slope_rad > max_allowed_slope_) {
    return std::numeric_limits<double>::infinity();
  }

  const double distance_3d = std::sqrt(dx * dx + dy * dy + dz * dz);

  const double slope_factor = 1.0 + std::pow(slope_rad / max_allowed_slope_, 2.0);

  const double final_cost = distance_3d * slope_factor;

  return cost_cache_[key] = final_cost;
}

void GridMapRRTStarPlanner::clear_cost_cache()
{
  cost_cache_.clear();
  cached_map_width_ = 0;
  cached_map_height_ = 0;
  last_map_timestamp_ = 0;
}

uint32_t GridMapRRTStarPlanner::flat_index(const grid_map::Index & idx, uint32_t width)
{
  return idx[0] + idx[1] * width;
}

uint64_t GridMapRRTStarPlanner::edge_key(uint32_t a_flat, uint32_t b_flat)
{
  return (static_cast<uint64_t>(a_flat) << 32) | b_flat;
}


bool GridMapRRTStarPlanner::check_goal_changed(const geometry_msgs::msg::Pose & goal_pose)
{
    // tolerance used for comparing goal positions
  const double tolerance = goal_threshold_;

    // check if the goal has changed significantly
  bool goal_changed = (std::abs(goal_pose.position.x - last_goal_pose_.position.x) > tolerance ||
    std::abs(goal_pose.position.y - last_goal_pose_.position.y) > tolerance ||
    std::abs(goal_pose.position.z - last_goal_pose_.position.z) > tolerance);

  if (goal_changed) {
    last_goal_pose_ = goal_pose;     // update last known goal
  }

  return goal_changed;
}

// Replace entire update() function:
void GridMapRRTStarPlanner::update(NavState & nav_state)
{
    // initial validations
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map")) {
    RCLCPP_DEBUG(get_node()->get_logger(), "goals, robot_pose or map missing. Returning");
    return;
  }

  const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "goals empty. Returning empty path");
    current_path_.poses.clear();
    nav_state.set("path", current_path_);
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal_pose = goals.goals.front().pose;
  const auto & map = nav_state.get<grid_map::GridMap>("map");

    // evaluate conditions once
  bool goal_changed = check_goal_changed(goal_pose);
  bool robot_deviated = false;
  bool path_too_short = current_path_.poses.size() < 3;
  double lateral_distance = 0.0;

    // check robot deviation only if a path exists
  if (!current_path_.poses.empty()) {
    lateral_distance = calculate_lateral_distance_to_path(current_path_, robot_pose.pose.pose);
    robot_deviated = (lateral_distance > max_lateral_deviation_);

    if (robot_deviated) {
      RCLCPP_INFO(get_node()->get_logger(),
                        "Robot deviated from path (%.2f m > %.2f m threshold), replanning",
                        lateral_distance, max_lateral_deviation_);
    } else {
      RCLCPP_DEBUG(get_node()->get_logger(),
                         "Robot deviation: %.2f m (within %.2f m threshold)",
                         lateral_distance, max_lateral_deviation_);
    }
  }

    // decide once whether to replan
  bool needs_replan = goal_changed || robot_deviated || path_too_short;
  bool needs_new_path = needs_replan || current_path_.poses.empty();

    // perform actions based on decision
  if (needs_replan) {
    if (goal_changed) {
      RCLCPP_INFO(get_node()->get_logger(), "New goal detected, regenerating path");
      last_goal_pose_ = goal_pose;
    } else if (robot_deviated) {
      RCLCPP_INFO(get_node()->get_logger(), "Robot deviated, regenerating path");
    } else if (path_too_short) {
      RCLCPP_DEBUG(get_node()->get_logger(),
                         "Path too short (%zu waypoints), regenerating",
                         current_path_.poses.size());
    }

        // reset state for replanning
    current_path_.poses.clear();
    clear_cost_cache();
    current_path_cost_ = std::numeric_limits<double>::max();
    kd_tree_ = std::make_unique<KDTree>();
  }

    // generate a new path if needed
  if (needs_new_path) {
    auto new_poses = generate_new_path(map, robot_pose.pose.pose, goal_pose);
    if (new_poses.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to generate a new path");
      return;
    }

    nav_msgs::msg::Path new_path;
    new_path.header.stamp = get_node()->now();
    new_path.header.frame_id = goals.header.frame_id;

    for (const auto & pose : new_poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = new_path.header;
      ps.pose = pose;
      new_path.poses.push_back(ps);
    }

        // accept or compare new path
    if (needs_replan) {
      current_path_ = new_path;
      current_path_cost_ = new_path_cost_;
      RCLCPP_DEBUG(get_node()->get_logger(),
                         "Using new path: cost %.2f, length %.2f m",
                         new_path_cost_, compute_path_length(new_path));
    } else if (current_path_.poses.empty()) {
      current_path_ = new_path;
      current_path_cost_ = new_path_cost_;
      RCLCPP_DEBUG(get_node()->get_logger(),
                         "Using new path (no previous path): cost %.2f, length %.2f m",
                         new_path_cost_, compute_path_length(new_path));
    } else {
      const double improvement_threshold = 5 * goal_threshold_;

      if (current_path_cost_ - new_path_cost_ > improvement_threshold) {
        double cost_improvement = current_path_cost_ - new_path_cost_;
        double old_cost = current_path_cost_;

        current_path_ = new_path;
        current_path_cost_ = new_path_cost_;

        RCLCPP_DEBUG(get_node()->get_logger(),
                             "Updated to better cost path: %.2f -> %.2f (improvement: %.2f)",
                             old_cost, new_path_cost_, cost_improvement);
      } else {
        double cost_difference = new_path_cost_ - current_path_cost_;
        RCLCPP_DEBUG(get_node()->get_logger(),
                             "Keeping current path: cost %.2f < %.2f (difference: %.2f, threshold: %.2f)",
                             current_path_cost_, new_path_cost_, cost_difference,
            improvement_threshold);
      }
    }
  }

    // apply trimming and publish
  if (!current_path_.poses.empty()) {
    size_t original_size = current_path_.poses.size();
    current_path_ = trim_path_from_robot(current_path_, robot_pose.pose.pose);

    if (current_path_.poses.size() != original_size) {
      RCLCPP_DEBUG(get_node()->get_logger(),
                         "Path trimmed: %zu -> %zu waypoints",
                         original_size, current_path_.poses.size());
    }
  }

  nav_state.set("path", current_path_);

  if (path_pub_->get_subscription_count() > 0) {
    path_pub_->publish(current_path_);
  }

  if (marker_pub_->get_subscription_count() > 0) {
    publish_path_markers(current_path_);
  }
}

std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::generate_new_path(
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::Pose & goal_pose)
{
  std::vector<geometry_msgs::msg::Pose> new_poses;

    // ensure goal is inside map bounds
  grid_map::Index goal_idx;
  if (!map.getIndex(grid_map::Position(goal_pose.position.x, goal_pose.position.y), goal_idx)) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal is outside the map bounds");
    return new_poses;
  }

    // always plan from robot pose for consistency
  RCLCPP_DEBUG(get_node()->get_logger(), "Generating new path from robot pose");
  new_poses = rrt_star(map, robot_pose, goal_pose);

  return new_poses;
}

void GridMapRRTStarPlanner::publish_path_markers(const nav_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray marker_array;

    // clear previous markers
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

    // create markers for each path pose
  int id = 0;
  for (const auto & pose_stamped : path.poses) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "rrt_star_path";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

        // set marker pose
    marker.pose = pose_stamped.pose;

        // set marker scale
    marker.scale.x = 0.3;      // arrow length
    marker.scale.y = 0.05;     // arrow thickness
    marker.scale.z = 0.05;

        // set marker color
    marker.color.r = 0.1f;
    marker.color.g = 0.1f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker_array.markers.push_back(marker);
  }

    // publish markers
  marker_pub_->publish(marker_array);
}
std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::smooth_path(
  const std::vector<geometry_msgs::msg::Pose> & input_path,
  int interpolation_points_per_segment)
{
    // Return the input path if it's too short to smooth
  if (input_path.size() < 3) {
    return input_path;
  }

  std::vector<geometry_msgs::msg::Pose> smoothed;
  const size_t N = input_path.size();
  const size_t preserve_n = std::max(1, final_poses_with_goal_orientation_);
  const double max_yaw_step = M_PI / 8.0;

    // Precompute positions and tangents
  std::vector<grid_map::Position> pts(N);
  std::vector<double> z_values(N);
  for (size_t i = 0; i < N; ++i) {
    pts[i] = {input_path[i].position.x, input_path[i].position.y};
    z_values[i] = input_path[i].position.z;
  }

  auto compute_tangent = [&](size_t idx) -> grid_map::Position {
      if (idx == 0) {
        return 0.5 * (pts[1] - pts[0]);
      } else if (idx + 1 >= N) {
        return 0.5 * (pts[N - 1] - pts[N - 2]);
      } else {
        return 0.5 * (pts[idx + 1] - pts[idx - 1]);
      }
    };

  std::vector<grid_map::Position> tangents(N);
  std::vector<double> z_tangents(N);
  for (size_t i = 0; i < N; ++i) {
    tangents[i] = compute_tangent(i);
    z_tangents[i] = (i == 0) ? 0.5 * (z_values[1] - z_values[0]) :
      (i + 1 >= N) ? 0.5 * (z_values[N - 1] - z_values[N - 2]) :
      0.5 * (z_values[i + 1] - z_values[i - 1]);
  }

    // Add the first point with corrected orientation
  geometry_msgs::msg::Pose first_pose = input_path.front();

    // Calculate orientation for the first point based on direction to second point
  if (input_path.size() > 1) {
    double dx = input_path[1].position.x - first_pose.position.x;
    double dy = input_path[1].position.y - first_pose.position.y;
    if (std::hypot(dx, dy) > 1e-6) {
      double yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      first_pose.orientation = tf2::toMsg(q);
    }
  }

  smoothed.push_back(first_pose);

    // Interpolate between points
  for (size_t i = 0; i < N - 1; ++i) {
    const auto & p0 = pts[i], & p1 = pts[i + 1];
    const auto & m0 = tangents[i], & m1 = tangents[i + 1];
    double z0 = z_values[i], z1 = z_values[i + 1];
    double z_m0 = z_tangents[i], z_m1 = z_tangents[i + 1];

    for (int s = 1; s <= interpolation_points_per_segment; ++s) {
      double t = static_cast<double>(s) / (interpolation_points_per_segment + 1);
      double t2 = t * t, t3 = t2 * t;

            // Hermite interpolation
      double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
      double h10 = t3 - 2.0 * t2 + t;
      double h01 = -2.0 * t3 + 3.0 * t2;
      double h11 = t3 - t2;

      geometry_msgs::msg::Pose sample;
      sample.position.x = h00 * p0.x() + h10 * m0.x() + h01 * p1.x() + h11 * m1.x();
      sample.position.y = h00 * p0.y() + h10 * m0.y() + h01 * p1.y() + h11 * m1.y();
      sample.position.z = h00 * z0 + h10 * z_m0 + h01 * z1 + h11 * z_m1;

            // Calculate orientation based on path direction
      double yaw = 0.0;
      if (!smoothed.empty()) {
        const auto & prev = smoothed.back().position;
        double dx = sample.position.x - prev.x;
        double dy = sample.position.y - prev.y;
        if (std::hypot(dx, dy) > 1e-6) {
          yaw = std::atan2(dy, dx);

                    // Smooth yaw transitions
          tf2::Quaternion prev_q;
          tf2::fromMsg(smoothed.back().orientation, prev_q);
          double pr, pp, prev_yaw;
          tf2::Matrix3x3(prev_q).getRPY(pr, pp, prev_yaw);

          double dyaw = yaw - prev_yaw;
          while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
          while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}

          if (std::abs(dyaw) > max_yaw_step) {
            yaw = prev_yaw + (dyaw > 0 ? max_yaw_step : -max_yaw_step);
          }
        } else {
                    // Keep previous orientation if no significant movement
          tf2::Quaternion prev_q;
          tf2::fromMsg(smoothed.back().orientation, prev_q);
          double pr, pp;
          tf2::Matrix3x3(prev_q).getRPY(pr, pp, yaw);
        }
      }

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      sample.orientation = tf2::toMsg(q);

            // Avoid adding points too close
      const auto & last_pos = smoothed.back().position;
      if (std::hypot(sample.position.x - last_pos.x,
                           sample.position.y - last_pos.y) >= spacing_)
      {
        smoothed.push_back(sample);
      }
    }

        // Add the next raw waypoint with corrected orientation
    geometry_msgs::msg::Pose next_pose = input_path[i + 1];

        // Calculate orientation for the waypoint based on path direction
    if (!smoothed.empty()) {
      const auto & prev = smoothed.back().position;
      double dx = next_pose.position.x - prev.x;
      double dy = next_pose.position.y - prev.y;
      if (std::hypot(dx, dy) > 1e-6) {
        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        next_pose.orientation = tf2::toMsg(q);
      }
    }

    smoothed.push_back(next_pose);
  }

    // Ensure the final `preserve_n` points have the goal orientation
  tf2::Quaternion goal_orientation;
  tf2::fromMsg(input_path.back().orientation, goal_orientation);

    // Apply goal orientation to the last preserve_n points
  size_t start_preserve = smoothed.size() > preserve_n ? smoothed.size() - preserve_n : 0;
  for (size_t i = start_preserve; i < smoothed.size(); ++i) {
    smoothed[i].orientation = tf2::toMsg(goal_orientation);
  }
  if (!smoothed.empty()) {
    smoothed.back().orientation = tf2::toMsg(goal_orientation);
  }
  RCLCPP_DEBUG(get_node()->get_logger(),
                 "Smoothed path: %zu -> %zu points (preserved %zu final orientations)",
                 input_path.size(), smoothed.size(), preserve_n);

  return smoothed;
}

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GridMapRRTStarPlanner, easynav::PlannerMethodBase)
