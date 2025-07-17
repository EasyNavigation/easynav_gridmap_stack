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
/// \brief Implementation of the GridmapMapsManager class.

#include <expected>

#include "grid_map_ros/grid_map_ros.hpp"

#include "easynav_gridmap_maps_manager/GridmapMapsManager.hpp"
#include "easynav_gridmap_maps_manager/utils.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "easynav_common/YTSession.hpp"

namespace easynav
{

using std::placeholders::_1;

GridmapMapsManager::GridmapMapsManager()
{
  NavState::register_printer<grid_map::GridMap>(
    [](const grid_map::GridMap & map) {
      std::string ret = "Gridmap of (" +
      std::to_string(map.getLength().x()) + " x " +
      std::to_string(map.getLength().y()) + ") with resolution " +
      std::to_string(map.getResolution());
      return ret;
    });
}

GridmapMapsManager::~GridmapMapsManager()
{
}


std::expected<void, std::string>
GridmapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);

  if (package_name != "" && map_path_file != "") {
    std::string pkgpath;
    try {
      pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + "/" + map_path_file;
    } catch(ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (!load_gridmap(map_path_, map_)) {
      return std::unexpected("File [" + map_path_ + "] not found or read error");
    }
  }

  gridmap_pub_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
    node->get_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable());

  incoming_map_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    node->get_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](grid_map_msgs::msg::GridMap::UniquePtr msg) {
      grid_map::GridMap incoming;
      if (grid_map::GridMapRosConverter::fromMessage(*msg, incoming, {"elevation"})) {
        map_need_update_ = true;
        map_ = incoming;
        gridmap_msg_ = *msg;
        gridmap_msg_.header.stamp = this->get_node()->now();
        gridmap_pub_->publish(gridmap_msg_);
      } else {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Error receiving gridmap");
      }
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      if (!save_gridmap(map_path_, map_)) {
        response->success = false;
        response->message = "Failed to save map to: " + map_path_;
      } else {
        response->success = true;
        response->message = "Map successfully saved to: " + map_path_;
      }
    });

    gridmap_msg_ = *grid_map::GridMapRosConverter::toMessage(map_);
    gridmap_msg_.header.stamp = this->get_node()->now();
    gridmap_pub_->publish(gridmap_msg_);

  return {};
}

void
GridmapMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  if (map_need_update_) {
    nav_state.set("map", map_);
    map_need_update_ = false;
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GridmapMapsManager, easynav::MapsManagerBase)
