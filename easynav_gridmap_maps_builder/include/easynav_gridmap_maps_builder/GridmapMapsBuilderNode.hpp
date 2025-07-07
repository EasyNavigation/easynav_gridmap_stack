// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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
/// \brief Definition of the GridmapMapsBuilderNode class.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDERNODE_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/**
 * @class GridmapMapsBuilderNode
 * @brief Lifecycle node that subscribes to point cloud sensor data and builds a grid map.
 *
 * This node processes perception data (point clouds) to generate a single outdoor grid map.
 * It uses ROS 2 lifecycle management for clean startup, activation, deactivation, and cleanup.
 * The node publishes the resulting grid map for downstream use.
 */
class GridmapMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GridmapMapsBuilderNode)

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Options for node initialization.
   */
  explicit GridmapMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  ~GridmapMapsBuilderNode();

  /**
   * @brief Lifecycle configure callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle activate callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle deactivate callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle cleanup callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Perform a processing cycle on the perception data and update the grid map.
   *
   * This method should be called periodically (e.g., in a timer or main loop) to process
   * incoming sensor data, update the internal grid map representation, and publish outputs.
   */
  void cycle();

  /**
   * @brief Registers a perception handler.
   * @param handler Shared pointer to a PerceptionHandler instance.
   */
  void register_handler(std::shared_ptr<PerceptionHandler> handler);

private:
  /// Name of the sensor topic to subscribe to (e.g., point clouds).
  std::string sensor_topic_;

  /// Map of perception data grouped by sensor name.
  std::map<std::string, std::vector<PerceptionPtr>> perceptions_;

  /// Callback group for concurrency management of subscriptions and timers.
  rclcpp::CallbackGroup::SharedPtr cbg_;

  /// Downsampling resolution applied to point cloud data.
  double downsample_resolution_;

  /// Default frame ID used for perception data and published messages.
  std::string perception_default_frame_;

  /// Publisher for the processed grid map.
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;

  /// Registered perception handlers by sensor name.
  std::map<std::string, std::shared_ptr<PerceptionHandler>> handlers_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDERNODE_HPP_
