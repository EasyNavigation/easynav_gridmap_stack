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

#include "grid_map_ros/grid_map_ros.hpp"

#include "easynav_gridmap_maps_manager/GridmapMapsBuilderNode.hpp"
#include "easynav_gridmap_maps_manager/utils.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{

GridmapMapsBuilderNode::GridmapMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gridmap_maps_builder_node", options),
  map_({"elevation"})
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  nav_state_ = std::make_shared<NavState>();

  if (!has_parameter("downsample_resolution")) {
    declare_parameter("downsample_resolution", 1.0);
  }
  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("perception_default_frame")) {
    declare_parameter("perception_default_frame", "map");
  }

  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "map_builder_gridmap/gridmap", 100);
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
  get_parameter("downsample_resolution", downsample_resolution_);
  get_parameter("perception_default_frame", perception_default_frame_);

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);

    RCLCPP_DEBUG(get_logger(),
                  "Loaded sensor parameters: id=%s topic=%s type=%s",
                  sensor_id.c_str(), topic.c_str(), msg_type.c_str());

    auto handler = std::make_shared<PointPerceptionHandler>();
    handler->initialize(shared_from_this(), cbg_, sensor_id);
    sensor_handlers_.push_back(handler);

    RCLCPP_DEBUG(get_logger(), "Creating perception for sensor %s", sensor_id.c_str());
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
  for (auto & handler : sensor_handlers_) {
    handler->cycle_rt(nav_state_);
  }

  auto point_perceptions = nav_state_->get_no_group<PointPerception>();
  if (point_perceptions.empty()) {
    return;
  }
  auto points = PointPerceptionsOpsView(point_perceptions).as_points();
  auto downsampled_points = PointPerceptionsOpsView(point_perceptions)
    .downsample(downsample_resolution_)
    .fuse(perception_default_frame_)
    .as_points();

  if (downsampled_points.empty()) {
    return;
  }

  map_.setFrameId(perception_default_frame_);
  map_.setTimestamp(point_perceptions[0]->stamp.nanoseconds());

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

  map_.setGeometry(grid_map::Length(length_x, length_y), resolution,
                      grid_map::Position(center_x, center_y));
  map_["elevation"].setConstant(0.0);

  // Set elevation
  for (const auto & pt : downsampled_points.points) {
    grid_map::Position pos(pt.x, pt.y);
    grid_map::Index index;
    if (map_.getIndex(pos, index)) {
      float & cell = map_.at("elevation", index);
      cell = pt.z;
    }
  }

  if (pub_->get_subscription_count() > 0) {
    auto msg = grid_map::GridMapRosConverter::toMessage(map_);
    pub_->publish(std::move(msg));
  }
}

} // namespace easynav
