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
/// \brief Implementation of the GridmapMapsBuilderNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include "easynav_gridmap_maps_builder/GridmapMapsBuilderNode.hpp"
#include "easynav_common/types/PointPerception.hpp"

namespace easynav
{

GridmapMapsBuilderNode::GridmapMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gridmap_maps_builder_node", options)
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!has_parameter("downsample_resolution")) {
    declare_parameter("downsample_resolution", 1.0);
  }
  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("perception_default_frame")) {
    declare_parameter("perception_default_frame", "map");
  }

  if (!has_parameter("smooth_elevation.sigma_spatial")) {
    declare_parameter("smooth_elevation.sigma_spatial", 1.0);
  }

  if (!has_parameter("smooth_elevation.sigma_intensity")) {
    declare_parameter("smooth_elevation.sigma_intensity", 0.1);
  }

  if (!has_parameter("compute_slope.radius")) {
    declare_parameter("compute_slope.radius", 1.25);
  }

  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "map_builder_gridmap/gridmap", rclcpp::QoS(1).transient_local().reliable());

  register_handler(std::make_shared<PointPerceptionHandler>());
}

GridmapMapsBuilderNode::~GridmapMapsBuilderNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
GridmapMapsBuilderNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  if (sensors.empty()) {
    RCLCPP_WARN(get_logger(), "No sensors configured in parameter.");
  }
  get_parameter("downsample_resolution", downsample_resolution_);
  get_parameter("perception_default_frame", perception_default_frame_);
  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, group;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }
    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", group);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);
    get_parameter(sensor_id + ".group", group);

      // traversability params

    get_parameter("smooth_elevation.sigma_spatial", sigma_spatial_);
    get_parameter("smooth_elevation.sigma_intensity", sigma_intensity_);
    get_parameter("compute_slope.radius", slope_radius_);

    RCLCPP_DEBUG(get_logger(),
                   "Loaded sensor parameters: id=%s topic=%s type=%s group=%s",
                   sensor_id.c_str(), topic.c_str(), msg_type.c_str(), group.c_str());

    auto handler_it = handlers_.find(group);
    if (handler_it == handlers_.end()) {
      RCLCPP_WARN(get_logger(), "No handler for group [%s]", group.c_str());
      continue;
    }

    auto ptr = handler_it->second->create(sensor_id);
    auto sub = handler_it->second->create_subscription(*this, topic, msg_type, ptr, cbg_);

    perceptions_[group].emplace_back(PerceptionPtr{ptr, sub});

    RCLCPP_DEBUG(get_logger(), "Creating perception for sensor %s", sensor_id.c_str());
    RCLCPP_DEBUG(get_logger(), "Handler group = %s", group.c_str());
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
GridmapMapsBuilderNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
GridmapMapsBuilderNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
GridmapMapsBuilderNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

void GridmapMapsBuilderNode::cycle()
{
    // Finish cycle if no new perceptions
  if (std::none_of(perceptions_["points"].begin(), perceptions_["points"].end(),
    [](const auto & p)
    {return p.perception->new_data;}))
  {
    return;
  }

  if (pub_->get_subscription_count() > 0) {
    auto point_perceptions = get_point_perceptions(perceptions_["points"]);
    auto processed_perceptions = PointPerceptionsOpsView(point_perceptions);
      // Fuse perceptions if the frame_id is different from default and downsample
    if (!point_perceptions.empty() && point_perceptions[0] &&
      point_perceptions[0]->frame_id != perception_default_frame_)
    {
      processed_perceptions.downsample(downsample_resolution_).fuse(perception_default_frame_);
    } else {
      processed_perceptions.downsample(downsample_resolution_);
    }

    auto downsampled_points = processed_perceptions.as_points();
    if (downsampled_points.empty()) {
      return;
    }

    grid_map::GridMap map({"elevation"});
    map.setFrameId(perception_default_frame_);

    if (point_perceptions[0]->stamp.nanoseconds() != 0) {
      map.setTimestamp(point_perceptions[0]->stamp.nanoseconds());
    } else {
      map.setTimestamp(now().nanoseconds());
    }

      // Get Geometry from PCL Cloud
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min();

    for (const auto & pt : downsampled_points.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
    }

    float resolution = 1;
    float length_x = max_x - min_x;
    float length_y = max_y - min_y;
    float center_x = (max_x + min_x) / 2.0;
    float center_y = (max_y + min_y) / 2.0;

    map.setGeometry(grid_map::Length(length_x, length_y), resolution,
                      grid_map::Position(center_x, center_y));
    map["elevation"].setConstant(0.0);   // Initialize elevations of all cells to zero

      // Set elevation
    for (const auto & pt : downsampled_points.points) {
      grid_map::Position pos(pt.x, pt.y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        float & cell = map.at("elevation", index);
        if (std::isnan(cell)) {
          cell = pt.z;
        } else {
          cell = (cell + pt.z) / 2.0;
        }
      }
    }
    smoothElevation(map, "elevation", "smoothed_elevation", sigma_spatial_, sigma_intensity_);
    computeSlope(map, "smoothed_elevation", "slope", slope_radius_);
    computeTraversability(map, "slope", "traversability");

    auto msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_->publish(std::move(msg));

      // Mark perceptions as not new after published
    for (auto & p : perceptions_["points"]) {
      if (p.perception->new_data) {
        p.perception->new_data = false;
      }
    }
  }
}

void
GridmapMapsBuilderNode::register_handler(std::shared_ptr<PerceptionHandler> handler)
{
  handlers_[handler->group()] = handler;
}

void
GridmapMapsBuilderNode::smoothElevation(
  grid_map::GridMap & map,
  const std::string & input_layer,
  const std::string & output_layer,
  double sigma_spatial,
  double sigma_intensity)
{
  const int half_kernel = 3;

  if (!map.exists(input_layer)) {
    RCLCPP_WARN(get_logger(), "Layer %s does not exist", input_layer.c_str());
    return;
  }

  if (!map.exists(output_layer)) {
    map.add(output_layer, std::numeric_limits<float>::quiet_NaN());
  }

  const double resolution = map.getResolution();

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Index currentIndex(*it);

    if (!map.isValid(currentIndex, input_layer)) {
      map.at(output_layer, currentIndex) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    float center_val = map.at(input_layer, currentIndex);
    if (std::isnan(center_val)) {
      map.at(output_layer, currentIndex) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    double weighted_sum = 0.0;
    double weight_total = 0.0;

    for (int dx = -half_kernel; dx <= half_kernel; ++dx) {
      for (int dy = -half_kernel; dy <= half_kernel; ++dy) {
        grid_map::Index neighborIndex(currentIndex(0) + dx, currentIndex(1) + dy);

        if (!map.isValid(neighborIndex, input_layer)) {
          continue;
        }

        float neighbor_val = map.at(input_layer, neighborIndex);
        if (std::isnan(neighbor_val)) {
          continue;
        }

        double spatial_dist = resolution * std::sqrt(dx * dx + dy * dy);
        double w_spatial = std::exp(-(spatial_dist * spatial_dist) /
            (2 * sigma_spatial * sigma_spatial));

        double intensity_diff = neighbor_val - center_val;
        double w_intensity = std::exp(-(intensity_diff * intensity_diff) /
            (2 * sigma_intensity * sigma_intensity));

        double weight = w_spatial * w_intensity;

        weighted_sum += neighbor_val * weight;
        weight_total += weight;
      }
    }

    if (weight_total > 0.0) {
      map.at(output_layer, currentIndex) = static_cast<float>(weighted_sum / weight_total);
    } else {
      map.at(output_layer, currentIndex) = center_val;
    }
  }
}
void GridmapMapsBuilderNode::computeSlope(
  grid_map::GridMap & map,
  const std::string & input_layer,
  const std::string & slope_layer,
  double radius)
{
  if (!map.exists(input_layer)) {
    RCLCPP_WARN(get_logger(), "Layer %s does not exist", input_layer.c_str());
    return;
  }

  if (!map.exists(slope_layer)) {
    map.add(slope_layer, std::numeric_limits<float>::quiet_NaN());
  }

  double resolution = map.getResolution();
  if (radius < resolution) {
    RCLCPP_WARN(get_logger(),
                  "Radius (%f) is smaller than map resolution (%f). This may lead to incorrect normal calculations.",
                  radius, resolution);
  }
  int max_offset = static_cast<int>(std::ceil(radius / resolution));

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Index center_index(*it);

    if (!map.isValid(center_index, input_layer)) {
      map.at(slope_layer, center_index) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    grid_map::Position center_pos;
    map.getPosition(center_index, center_pos);

    for (int dx = -max_offset; dx <= max_offset; ++dx) {
      for (int dy = -max_offset; dy <= max_offset; ++dy) {
        grid_map::Index neighbor_index(center_index(0) + dx, center_index(1) + dy);

        if (!map.isValid(neighbor_index, input_layer)) {
          continue;
        }

        float height = map.at(input_layer, neighbor_index);
        if (std::isnan(height)) {
          continue;
        }

        grid_map::Position pos;
        map.getPosition(neighbor_index, pos);

        double dist = (pos - center_pos).norm();
        if (dist > radius) {
          continue;
        }

        points.emplace_back(pos.x(), pos.y(), height);
      }
    }

    if (points.size() < 3) {
      map.at(slope_layer, center_index) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

      // Compute surface normal via PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : points) {
      centroid += p;
    }
    centroid /= points.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto & p : points) {
      Eigen::Vector3d diff = p - centroid;
      cov += diff * diff.transpose();
    }
    cov /= points.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);   // smallest eigenvalue

    if (normal.z() < 0) {
      normal = -normal;
    }

    float slope = std::acos(std::clamp(normal.z(), -1.0, 1.0));

      // filter too high slopes
    if (slope > M_PI / 2) {
      slope = std::numeric_limits<float>::quiet_NaN();
    }

    map.at(slope_layer, center_index) = slope;
  }
}

void GridmapMapsBuilderNode::computeTraversability(
  grid_map::GridMap & map,
  const std::string & slope_layer,
  const std::string & output_layer)
{
  if (!map.exists(slope_layer)) {
    RCLCPP_WARN(get_logger(), "Layer %s does not exist", slope_layer.c_str());
    return;
  }

  if (!map.exists(output_layer)) {
    map.add(output_layer, std::numeric_limits<float>::quiet_NaN());
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Index index(*it);

    if (!map.isValid(index, slope_layer)) {
      map.at(output_layer, index) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    float slope = map.at(slope_layer, index);
    if (std::isnan(slope)) {
      map.at(output_layer, index) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    float value = 1.0f / (1.0f + std::exp(10.0f * (slope - 0.7f)));
    int val = static_cast<int>(value * 3.0f);
    val = std::clamp(val, 0, 2);

    constexpr std::array<float, 3> traversability = {0.0f, 0.5f, 1.0f};
    map.at(output_layer, index) = traversability[val];
  }
}
} // namespace easynav
