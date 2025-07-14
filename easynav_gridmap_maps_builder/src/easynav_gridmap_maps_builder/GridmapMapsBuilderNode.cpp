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

#include "yaml-cpp/yaml.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "easynav_gridmap_maps_builder/GridmapMapsBuilderNode.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

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

  std::string package_name, map_path_file;
  if (!has_parameter("package")) {
    declare_parameter("package", package_name);
  }

  if (!has_parameter("map_path_file")) {
    declare_parameter("map_path_file", "map_path_file");
  }

//   if (package_name != "" && map_path_file != "") {
//     std::string pkgpath;
//     try {
//       pkgpath = ament_index_cpp::get_package_share_directory(package_name);
//       map_path_ = pkgpath + "/" + map_path_file;
//     } catch(ament_index_cpp::PackageNotFoundError & ex) {
//       return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
//     }
// 
//     if (!load_gridmap(map_path_, map_)) {
//       return std::unexpected("File [" + map_path_ + "] not found");
//     }
//   }

  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "map_builder_gridmap/gridmap", rclcpp::QoS(1).transient_local().reliable());

//   savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
//     node->get_name() + std::string("/") + plugin_name + "/savemap",
//     [this](
//       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//     {
//       (void)request;
//       if (!save_gridmap(map_path_, map_)) {
//         response->success = false;
//         response->message = "Failed to save map to: " + map_path_;
//       } else {
//         response->success = true;
//         response->message = "Map successfully saved to: " + map_path_;
//       }
//     });
// 
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


bool
GridmapMapsBuilderNode::save_gridmap(const std::string & filename, const grid_map::GridMap & map)
{
  std::string dir = std::filesystem::path(filename).parent_path();
  YAML::Emitter yaml;
  yaml << YAML::BeginMap;
  yaml << YAML::Key << "layers" << YAML::Value << YAML::BeginSeq;

  for (const std::string & layer : map.getLayers()) {
    // Compute min and max values of the layer
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      if (!map.isValid(*it, layer)) continue;
      float value = map.at(layer, *it);
      min_val = std::min(min_val, value);
      max_val = std::max(max_val, value);
    }

    // Save layer image
    cv::Mat image;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layer, CV_8UC1, min_val, max_val, image);
    std::string image_filename = layer + ".pgm";
    std::string image_path = dir + "/" + image_filename;
    cv::imwrite(image_path, image);

    yaml << YAML::BeginMap;
    yaml << YAML::Key << "name" << YAML::Value << layer;
    yaml << YAML::Key << "min" << YAML::Value << min_val;
    yaml << YAML::Key << "max" << YAML::Value << max_val;
    yaml << YAML::EndMap;
  }
  yaml << YAML::EndSeq;
  yaml << YAML::Key << "resolution" << YAML::Value << map.getResolution();
  yaml << YAML::Key << "length" << YAML::Value << YAML::Flow << std::vector<double>{map.getLength().x(), map.getLength().y()};
  yaml << YAML::Key << "position" << YAML::Value << YAML::Flow << std::vector<double>{map.getPosition().x(), map.getPosition().y()};
  yaml << YAML::EndMap;

  std::ofstream file_out(filename);
  if (!file_out) return false;
  file_out << yaml.c_str();
  return true;
}

bool
GridmapMapsBuilderNode::load_gridmap(const std::string & filename, grid_map::GridMap & map)
{
  std::ifstream file_in(filename);
  if (!file_in) return false;

  YAML::Node config = YAML::LoadFile(filename);
  if (!config["layers"]) return false;

  map.setFrameId("map");

  auto resolution = config["resolution"].as<double>();
  auto length = config["length"].as<std::vector<double>>();
  auto position = config["position"].as<std::vector<double>>();

  map.setGeometry(grid_map::Length(length[0], length[1]), resolution, grid_map::Position(position[0], position[1]));

  std::string dir = std::filesystem::path(filename).parent_path();

  for (const auto & layer_node : config["layers"]) {
    std::string layer_name = layer_node["name"].as<std::string>();
    float min_val = layer_node["min"].as<float>();
    float max_val = layer_node["max"].as<float>();

    std::string image_path = dir + "/" + layer_name + ".pgm";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) return false;

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, layer_name, map, min_val, max_val);
  }

  return true;
}

} // namespace easynav
