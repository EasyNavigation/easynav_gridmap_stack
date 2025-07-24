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

#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <sstream>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "easynav_gridmap_astar_planner/GridMapAStarPlanner.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

struct IndexHash
{
  std::size_t operator()(const grid_map::Index & index) const
  {
    return std::hash<int>()(index.x()) ^ (std::hash<int>()(index.y()) << 1);
  }
};

struct IndexEqual
{
  bool operator()(const grid_map::Index & a, const grid_map::Index & b) const
  {
    return (a == b).all();  // comparación elemento a elemento
  }
};

struct GridNode
{
  grid_map::Index index;
  double cost;
  double priority;
  bool operator>(const GridNode & other) const
  {
    return priority > other.priority;
  }
};

double
heuristic(
  const grid_map::GridMap & map, const grid_map::Index & a,
  const grid_map::Index & b)
{
  if (!map.isValid(a, "elevation") || !map.isValid(b, "elevation")) {
    return std::numeric_limits<double>::max();
  }

  grid_map::Position pa, pb;
  map.getPosition(a, pa);
  map.getPosition(b, pb);

  double dx = pb.x() - pa.x();
  double dy = pb.y() - pa.y();
  double dz = map.at("elevation", b) - map.at("elevation", a);

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<std::pair<int, int>> neighbors8 = {
  {-1, -1}, {-1, 0}, {-1, 1},
  {0, -1}, {0, 1},
  {1, -1}, {1, 0}, {1, 1}
};

double
compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;

  if (path.poses.size() < 2) {return 0.0;}

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  return total_length;
}

GridMapAStarPlanner::GridMapAStarPlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>([](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;
      ret << "Path with " << path.poses.size() << " poses and length "
          << compute_path_length(path) << " m.";
      return ret.str();
  });
}

std::expected<void, std::string> GridMapAStarPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  double max_allowed_slope_deg = 30.0;
  node->declare_parameter<double>(plugin_name + ".robot_radius", 0.3);
  node->declare_parameter<double>(plugin_name + ".clearance_distance", 0.2);
  node->declare_parameter(plugin_name + ".max_allowed_slope_deg", max_allowed_slope_deg);

  node->get_parameter(plugin_name + ".robot_radius", robot_radius_);
  node->get_parameter(plugin_name + ".clearance_distance", clearance_distance_);
  node->get_parameter(plugin_name + ".max_allowed_slope_deg", max_allowed_slope_deg);

  max_allowed_slope_ = max_allowed_slope_deg * M_PI / 180.0;

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planner/path", 10);

  get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);

  return {};
}

void
GridMapAStarPlanner::update(NavState & nav_state)
{
//   RCLCPP_DEBUG(
//     get_node()->get_logger(),
//     "GridMapAStarPlanner::update starting with navstate: \n%s\n",
//     nav_state.debug_string().c_str());

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

  const auto robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & map = nav_state.get<grid_map::GridMap>("map");

  auto poses = a_star_path(map, robot_pose.pose.pose, goal);

  if (!poses.empty()) {
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;

    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = current_path_.header;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }

    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }

  nav_state.set("path", current_path_);
}

bool GridMapAStarPlanner::isTraversable(
  const grid_map::GridMap & map,
  const grid_map::Index & from,
  const grid_map::Index & to) const
{
  grid_map::Position pos_from, pos_to;
  map.getPosition(from, pos_from);
  map.getPosition(to, pos_to);
  if (!map.isInside(pos_from) || !map.isInside(pos_to)) {return false;}
  if (!map.isValid(from, "elevation") || !map.isValid(to, "elevation")) {
    return false;
  }

  double z1 = map.at("elevation", from);
  double z2 = map.at("elevation", to);
  double dx = map.getResolution() * (to.x() - from.x());
  double dy = map.getResolution() * (to.y() - from.y());
  double dz = z2 - z1;

  double slope_rad = std::atan2(std::abs(dz), std::hypot(dx, dy));
  return slope_rad <= max_allowed_slope_;
}

std::vector<geometry_msgs::msg::Pose> GridMapAStarPlanner::a_star_path(
  const grid_map::GridMap & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open;
  std::unordered_map<grid_map::Index, grid_map::Index, IndexHash, IndexEqual> came_from;
  std::unordered_map<grid_map::Index, double, IndexHash, IndexEqual> cost_so_far;

  grid_map::Index start_index, goal_index;
  if (!map.getIndex(grid_map::Position(start.position.x, start.position.y), start_index)) {
    return {};
  }
  if (!map.getIndex(grid_map::Position(goal.position.x, goal.position.y), goal_index)) {
    return {};
  }

  open.push({start_index, 0.0, heuristic(map, start_index, goal_index)});
  cost_so_far[start_index] = 0.0;


  for (; !open.empty(); ) {
    GridNode current = open.top(); open.pop();
    if ((current.index == goal_index).all()) {break;}

    for (auto [dx, dy] : neighbors8) {
      grid_map::Index neighbor = current.index + grid_map::Index(dx, dy);
      grid_map::Position pos_neighbor;
      map.getPosition(neighbor, pos_neighbor);
      if (!map.isInside(pos_neighbor)) {continue;}
      if (!isTraversable(map, current.index, neighbor)) {continue;}

      double new_cost = cost_so_far[current.index] + heuristic(map, current.index, neighbor);
      if (!cost_so_far.contains(neighbor) || new_cost < cost_so_far[neighbor]) {
        cost_so_far[neighbor] = new_cost;
        double priority = new_cost + heuristic(map, neighbor, goal_index);
        open.push({neighbor, new_cost, priority});
        came_from[neighbor] = current.index;
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose> path;
  grid_map::Index current = goal_index;
  while (came_from.contains(current)) {
    grid_map::Position pos;
    map.getPosition(current, pos);
    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = map.at("elevation", current);
    pose.orientation = goal.orientation;
    path.push_back(pose);
    current = came_from[current];
  }
  std::reverse(path.begin(), path.end());

  if (path.empty()) {path.push_back(goal);}

  return path;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GridMapAStarPlanner, easynav::PlannerMethodBase)
