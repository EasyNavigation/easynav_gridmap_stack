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


#include <fstream>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "easynav_gridmap_maps_manager/utils.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "yaml-cpp/yaml.h"

namespace easynav
{

bool save_gridmap(const std::string & filename, const grid_map::GridMap & map)
{
  std::string dir = std::filesystem::path(filename).parent_path();
  if (dir == "") {
    dir = ".";
  }

  YAML::Emitter yaml;
  yaml << YAML::BeginMap;
  yaml << YAML::Key << "layers" << YAML::Value << YAML::BeginSeq;

  for (const std::string & layer : map.getLayers()) {
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      if (!map.isValid(*it, layer)) {continue;}
      float value = map.at(layer, *it);
      min_val = std::min(min_val, value);
      max_val = std::max(max_val, value);
    }

    cv::Mat image;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      map, layer, CV_8UC1, min_val, max_val, image);
    std::string base_name = std::filesystem::path(filename).stem();
    std::string image_filename = base_name + "_" + layer + ".pgm";
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
  yaml << YAML::Key << "length" << YAML::Value << YAML::Flow <<
    std::vector<double>{map.getLength().x(), map.getLength().y()};
  yaml << YAML::Key << "position" << YAML::Value << YAML::Flow <<
    std::vector<double>{map.getPosition().x(), map.getPosition().y()};
  yaml << YAML::EndMap;

  std::ofstream file_out(filename);
  if (!file_out) {return false;}
  file_out << yaml.c_str();
  return true;
}

bool load_gridmap(const std::string & filename, grid_map::GridMap & map)
{
  std::ifstream file_in(filename);
  if (!file_in) {return false;}

  YAML::Node config = YAML::LoadFile(filename);
  if (!config["layers"]) {return false;}

  map.setFrameId("map");

  auto resolution = config["resolution"].as<double>();
  auto length = config["length"].as<std::vector<double>>();
  auto position = config["position"].as<std::vector<double>>();

  map.setGeometry(grid_map::Length(length[0], length[1]), resolution,
    grid_map::Position(position[0], position[1]));

  std::string dir = std::filesystem::path(filename).parent_path();
  if (dir == "") {
    dir = ".";
  }
  std::string base_name = std::filesystem::path(filename).stem();

  for (const auto & layer_node : config["layers"]) {
    std::string layer_name = layer_node["name"].as<std::string>();
    float min_val = layer_node["min"].as<float>();
    float max_val = layer_node["max"].as<float>();

    std::string image_path = dir + "/" + base_name + "_" + layer_name + ".pgm";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {return false;}

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
      image, layer_name, map, min_val, max_val);
  }

  return true;
}
}  // namespace easynav
