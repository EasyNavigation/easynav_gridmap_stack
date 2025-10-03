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
      ret           << "Path with " << path.poses.size() << " poses, length "
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

  node->get_parameter(plugin_name + ".max_allowed_slope_deg", max_allowed_slope_deg_);
  node->get_parameter(plugin_name + ".max_iters", max_iters_);
  node->get_parameter(plugin_name + ".step_size", step_size_);
  node->get_parameter(plugin_name + ".neighbor_radius", neighbor_radius_);
  node->get_parameter(plugin_name + ".goal_bias", goal_bias_);
  node->get_parameter(plugin_name + ".goal_threshold", goal_threshold_);
  node->get_parameter(plugin_name + ".kdtree_rebuild_interval", kdtree_rebuild_interval_);
  node->get_parameter(plugin_name + ".spacing", spacing_);

  max_allowed_slope_ = max_allowed_slope_deg_ * M_PI / 180.0;

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planner/path", 10);

  node->get_logger().set_level(rclcpp::Logger::Level::Debug);

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
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

  grid_map::Index start_idx, goal_idx;
  if (!map.getIndex(grid_map::Position(start.position.x, start.position.y), start_idx) ||
    !map.getIndex(grid_map::Position(goal.position.x, goal.position.y), goal_idx))
  {
    throw std::runtime_error("Start or goal position is outside map bounds");
  }

  auto root = std::make_shared<RRTNode>();
  root->index = start_idx;
  root->cost = 0.0;
  tree.push_back(root);

  std::shared_ptr<RRTNode> best_goal_node = nullptr;
  double best_goal_cost = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iters_; ++iter) {
    grid_map::Index rand_idx = (prob_dist(rng) < goal_bias_) ? goal_idx : random_index(map, rng);
    auto nearest = find_nearest(tree, rand_idx, map);
    if (!nearest) {
      continue;
    }

    auto new_idx = steer(map, nearest->index, rand_idx);
    if ((new_idx == grid_map::Index(-1, -1)).all()) {
      continue;
    }

    if (std::any_of(tree.begin(), tree.end(),
      [&new_idx](const std::shared_ptr<RRTNode> & node)
      {
        return (node->index == new_idx).all();
                            }))
    {
      continue;
    }
    double edge_cost = traversal_cost(map, nearest->index, new_idx);
    if (!std::isfinite(edge_cost)) {
      continue;
    }

    auto new_node = std::make_shared<RRTNode>();
    new_node->index = new_idx;
    new_node->cost = nearest->cost + edge_cost;
    new_node->parent = nearest;

            // RRT* optimization: try to choose a better parent from nearby nodes
    double rewire_radius_ = 2.0;
    for (const auto & near_node : tree) {
      if (distance(map, near_node->index, new_idx) > rewire_radius_) {
        continue;
      }

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

            // Rewiring: try to improve costs of nearby existing nodes
    for (auto & near_node : tree) {
      if (near_node == new_node) {
        continue;
      }
      if (distance(map, new_idx, near_node->index) > rewire_radius_) {
        continue;
      }

      double cost = traversal_cost(map, new_idx, near_node->index);
      if (!std::isfinite(cost)) {
        continue;
      }

      double total = new_node->cost + cost;
      if (total < near_node->cost) {
        near_node->cost = total;
        near_node->parent = new_node;
      }
    }

            // Check if new node is close enough to goal
    if (distance(map, new_idx, goal_idx) < goal_threshold_) {
      double goal_cost = traversal_cost(map, new_idx, goal_idx);
      if (std::isfinite(goal_cost)) {
        double total = new_node->cost + goal_cost;
        if (total < best_goal_cost) {
          best_goal_node = new_node;
          best_goal_cost = total;
        }
      }
    }
  }

  auto raw_path = extract_path(best_goal_node, map, goal);
  raw_path = smooth_path(raw_path, 5);

  return raw_path;
}

std::shared_ptr<RRTNode> GridMapRRTStarPlanner::find_nearest(
  const std::vector<std::shared_ptr<RRTNode>> & tree,
  const grid_map::Index & target,
  const grid_map::GridMap & map)
{
  if (tree.empty()) {
    return nullptr;
  }

  return *std::min_element(tree.begin(), tree.end(),
           [&](const auto & a, const auto & b)
           {
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

std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::extract_path(
  std::shared_ptr<RRTNode> goal_node,
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & goal)
{
        // Collect nodes from goal back to start by following parent links
  std::vector<std::shared_ptr<RRTNode>> nodes;
  nodes.reserve(100);

  for (auto current = goal_node; current; current = current->parent) {
    nodes.push_back(current);
  }
        // Reverse to get path from start to goal
  std::reverse(nodes.begin(), nodes.end());

        // Convert RRT nodes to ROS poses
  std::vector<geometry_msgs::msg::Pose> path;
  path.reserve(nodes.size());

  for (size_t i = 0; i < nodes.size(); ++i) {
    const auto & node = nodes[i];
    grid_map::Position pos;
    map.getPosition(node->index, pos);

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = map.at("elevation", node->index);         // Set z using elevation layer

            // Compute orientation if there is a next node
    if (i + 1 < nodes.size()) {
      grid_map::Position next_pos;
      map.getPosition(nodes[i + 1]->index, next_pos);

      double dx = next_pos.x() - pos.x();
      double dy = next_pos.y() - pos.y();
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.orientation = tf2::toMsg(q);
    }

    path.push_back(pose);
  }

  if (!path.empty()) {
    path.back().orientation = goal.orientation;
  }

  return path;
}

double GridMapRRTStarPlanner::traversal_cost(
  const grid_map::GridMap & map,
  const grid_map::Index & to,
  const grid_map::Index & from)
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

void GridMapRRTStarPlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();

  if (!nav_state.has("goals") || !nav_state.has("robot_pose") || !nav_state.has("map")) {
    RCLCPP_DEBUG(
                get_node()->get_logger(), "goals, robot_pose or map missing. Returning");
    return;
  }

  const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    RCLCPP_DEBUG(
                get_node()->get_logger(), "goals empty. Returning empty path");
    nav_state.set("path", current_path_);
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal_pose = goals.goals.front().pose;
  const auto & map = nav_state.get<grid_map::GridMap>("map");

  clear_cost_cache();
  kd_tree_ = std::make_unique<KDTree>();

  auto new_poses = rrt_star(map, robot_pose.pose.pose, goal_pose);

  if (!new_poses.empty()) {
    nav_msgs::msg::Path new_path;
    new_path.header.stamp = get_node()->now();
    new_path.header.frame_id = goals.header.frame_id;

    for (const auto & pose : new_poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = new_path.header;
      ps.pose = pose;
      new_path.poses.push_back(ps);
    }

    current_path_ = new_path;
    nav_state.set("path", current_path_);

    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }
}

std::vector<geometry_msgs::msg::Pose> GridMapRRTStarPlanner::smooth_path(
  const std::vector<geometry_msgs::msg::Pose> & input_path,
  int interpolation_points_per_segment)
{
  std::vector<geometry_msgs::msg::Pose> smoothed;

  if (input_path.size() < 4) {
    return input_path;
  }

  for (size_t i = 1; i < input_path.size() - 2; ++i) {
    const auto & p0 = input_path[i - 1].position;
    const auto & p1 = input_path[i].position;
    const auto & p2 = input_path[i + 1].position;
    const auto & p3 = input_path[i + 2].position;

    for (int j = 0; j < interpolation_points_per_segment; ++j) {
      double t = static_cast<double>(j) / interpolation_points_per_segment;
      geometry_msgs::msg::Pose pose;

                // Catmull–Rom spline
      pose.position.x = 0.5 * ((2 * p1.x) +
        (-p0.x + p2.x) * t +
        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t * t +
        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t * t * t);

      pose.position.y = 0.5 * ((2 * p1.y) +
        (-p0.y + p2.y) * t +
        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t * t +
        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t * t * t);

      pose.position.z = 0.5 * ((2 * p1.z) +
        (-p0.z + p2.z) * t +
        (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t * t +
        (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t * t * t);

      if (!smoothed.empty()) {
        const auto & prev = smoothed.back().position;
        double dx = pose.position.x - prev.x;
        double dy = pose.position.y - prev.y;

        if (std::hypot(dx, dy) > 1e-6) {
          double yaw = std::atan2(dy, dx);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.orientation = tf2::toMsg(q);
        } else {
          pose.orientation = smoothed.back().orientation;
        }
      } else {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation = tf2::toMsg(q);
      }

                // Avoid too much points, minimal spacing
      if (!smoothed.empty()) {
        const auto & prev = smoothed.back().position;
        if (std::hypot(pose.position.x - prev.x,
                                   pose.position.y - prev.y) < spacing_)
        {
          continue;
        }
      }

      smoothed.push_back(pose);
    }
  }

        // Add last original waypoint with correct orientation
  if (!input_path.empty()) {
    geometry_msgs::msg::Pose last = input_path.back();
    if (!smoothed.empty()) {
      const auto & prev = smoothed.back().position;
      double dx = last.position.x - prev.x;
      double dy = last.position.y - prev.y;
      if (std::hypot(dx, dy) > 1e-6) {
        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        last.orientation = tf2::toMsg(q);
      } else {
        last.orientation = smoothed.back().orientation;
      }
    }
    smoothed.push_back(last);
  }

  return smoothed;
}

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GridMapRRTStarPlanner, easynav::PlannerMethodBase)
