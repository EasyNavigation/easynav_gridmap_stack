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
/// \brief Definition of the GridmapManagerNode class.

#ifndef EASYNAV_GRIDMAP_MAPS_MANAGER__GRIDMAPMAPSMANAGER_HPP_
#define EASYNAV_GRIDMAP_MAPS_MANAGER__GRIDMAPMAPSMANAGER_HPP_

#include "grid_map_msgs/msg/grid_map.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "grid_map_ros/grid_map_ros.hpp"

#include "easynav_core/MapsManagerBase.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{


/**
 * @class GridmapMapsManager
 * @brief A plugin-based map manager using Gridmaps.
 *
 */
class GridmapMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  GridmapMapsManager();

  /**
   * @brief Destructor.
   */
  ~GridmapMapsManager();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the internal maps using the current navigation state.
   *
   * Intended to be called periodically. May perform dynamic map updates
   * based on new sensor data or internal state.
   *
   * @param nav_state Current state of the navigation system.
   */
  virtual void update(NavState & nav_state) override;

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief The Gridmap.
   */
  grid_map::GridMap map_;

  /**
   * @brief Publisher for the gridmap.
   */
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;

  /**
   * @brief Subscriber for external incoming gridmap updates.
   */
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr incoming_map_sub_;

  /**
   * @brief Service for saving current map to disk.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

  /**
   * @brief Cached gridmap message for the gridmap.
   */
  grid_map_msgs::msg::GridMap gridmap_msg_;

  bool map_need_update_ {true};
};

}  // namespace easynav

#endif  // EASYNAV_GRIDMAP_MAPS_MANAGER__GRIDMAPMAPSBUILDERNODE_HPP_
