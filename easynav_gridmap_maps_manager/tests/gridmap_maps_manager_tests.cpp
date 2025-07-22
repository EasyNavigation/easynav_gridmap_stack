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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_gridmap_maps_manager/GridmapMapsBuilderNode.hpp"
#include "easynav_gridmap_maps_manager/GridmapMapsManager.hpp"
#include "easynav_gridmap_maps_manager/utils.hpp"

class GridmapMapsMangerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

using namespace std::chrono_literals;

TEST_F(GridmapMapsMangerTest, test_save)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("sensors", std::vector<std::string>{"map"});
  options.append_parameter_override("downsample_resolution", 1.0);
  options.append_parameter_override("perception_default_frame", "map");
  options.append_parameter_override("map.topic", "map");
  options.append_parameter_override("map.type", "sensor_msgs/msg/PointCloud2");
  options.append_parameter_override("map.group", "points");
  std::vector<std::string> remappings = {
    "/map_builder_gridmap/gridmap:=/test_node/test2/incoming_map"
  };
  options.arguments(remappings);

  auto builder_node = std::make_shared<easynav::GridmapMapsBuilderNode>(options);
  auto test_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto manager = std::make_shared<easynav::GridmapMapsManager>();
  manager->initialize(test_node, "test2");

  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);

  grid_map_msgs::msg::GridMap::SharedPtr received_gridmap;
  auto sub = test_node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/test_node/test2/incoming_map", 100,
    [&] (const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      received_gridmap = msg;
    }
  );

  grid_map_msgs::msg::GridMap::SharedPtr emitted_gridmap;
  auto sub2 = test_node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/test_node/test2/map", rclcpp::QoS(1).transient_local().reliable(),
    [&] (const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      emitted_gridmap = msg;
    }
  );

  easynav::NavState navstate;

  builder_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(builder_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  builder_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(builder_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = test_node->now();
  cloud.height = 1;
  cloud.width = 10;
  cloud.fields.resize(3);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = -10.0 + i * 2.0;
      *iter_y = -10.0 + i * 2.0;

      if (*iter_x < -5.0) {
        *iter_z = -2.0;
      } else if (*iter_x < 0) {
        *iter_z = -1.0;
      } else if (*iter_x < 5) {
        *iter_z = 1.0;
      } else {
        *iter_z = 2.0;
      }
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(builder_node->get_node_base_interface());
  executor.add_node(test_node->get_node_base_interface());

  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    pub->publish(cloud);
    executor.spin_some();
    builder_node->cycle();
    manager->update(navstate);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  auto map = builder_node->get_map();
  ASSERT_TRUE(map.exists("elevation"));

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    ASSERT_EQ(cloud.width, 10);
    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      grid_map::Position pos(*iter_x, *iter_y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        auto z = map.at("elevation", index);

        if (*iter_x < -5.0) {
          EXPECT_NEAR(z, -2.0, 0.0001);
        } else if (*iter_x < 0) {
          EXPECT_NEAR(z, -1.0, 0.0001);
        } else if (*iter_x < 5) {
          EXPECT_NEAR(z, 1.0, 0.0001);
        } else {
          EXPECT_NEAR(z, 2.0, 0.0001);
        }
      }
    }
  }

  ASSERT_NE(received_gridmap, nullptr);
  grid_map::GridMap incoming;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*received_gridmap, incoming,
    {"elevation"}));

  ASSERT_EQ(map.getLength().x(), incoming.getLength().x());
  ASSERT_EQ(map.getLength().y(), incoming.getLength().y());
  ASSERT_EQ(map.getPosition().x(), incoming.getPosition().x());
  ASSERT_EQ(map.getPosition().y(), incoming.getPosition().y());
  ASSERT_EQ(map.getResolution(), incoming.getResolution());

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    map.getPosition(index, pos);

    double map_z = map.at("elevation", index);

    grid_map::Index index2;
    ASSERT_TRUE(incoming.getIndex(pos, index2));

    double incoming_z = incoming.at("elevation", index2);

    ASSERT_NEAR(map_z, incoming_z, 0.001);
  }

  ASSERT_EQ(received_gridmap->info, emitted_gridmap->info);
  ASSERT_EQ(received_gridmap->layers, emitted_gridmap->layers);
  ASSERT_EQ(received_gridmap->basic_layers, emitted_gridmap->basic_layers);
  ASSERT_EQ(received_gridmap->data, emitted_gridmap->data);

  auto test_node_srv = rclcpp::Node::make_shared("test_node_srv");
  auto client = test_node_srv->create_client<std_srvs::srv::Trigger>("/test_node/test2/savemap");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client->async_send_request(request);


  start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    pub->publish(cloud);
    executor.spin_some();
    rclcpp::spin_until_future_complete(test_node_srv, result_future);
    builder_node->cycle();
    manager->update(navstate);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  auto result = result_future.get();

  grid_map::GridMap map_file;
  ASSERT_TRUE(easynav::load_gridmap("/tmp/gridmap_map.yaml", map_file));


  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    map.getPosition(index, pos);

    double map_z = map.at("elevation", index);

    grid_map::Index index2;
    ASSERT_TRUE(map_file.getIndex(pos, index2));

    double incoming_z = map_file.at("elevation", index2);

    ASSERT_NEAR(map_z, incoming_z, 0.05);
  }
}

TEST_F(GridmapMapsMangerTest, test_modify_layers)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("sensors", std::vector<std::string>{"map"});
  options.append_parameter_override("downsample_resolution", 1.0);
  options.append_parameter_override("perception_default_frame", "map");
  options.append_parameter_override("map.topic", "map");
  options.append_parameter_override("map.type", "sensor_msgs/msg/PointCloud2");
  options.append_parameter_override("map.group", "points");
  std::vector<std::string> remappings = {
    "/map_builder_gridmap/gridmap:=/test_node/test2/incoming_map"
  };
  options.arguments(remappings);

  auto builder_node = std::make_shared<easynav::GridmapMapsBuilderNode>(options);
  auto test_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto manager = std::make_shared<easynav::GridmapMapsManager>();
  manager->initialize(test_node, "test2");

  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);

  grid_map_msgs::msg::GridMap::SharedPtr received_gridmap;
  auto sub = test_node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/test_node/test2/incoming_map", 100,
    [&] (const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      received_gridmap = msg;
    }
  );

  grid_map_msgs::msg::GridMap::SharedPtr emitted_gridmap;
  auto sub2 = test_node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/test_node/test2/map", rclcpp::QoS(1).transient_local().reliable(),
    [&] (const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      emitted_gridmap = msg;
    }
  );

  easynav::NavState navstate;

  builder_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(builder_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  builder_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(builder_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = test_node->now();
  cloud.height = 1;
  cloud.width = 10;
  cloud.fields.resize(3);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = -10.0 + i * 2.0;
      *iter_y = -10.0 + i * 2.0;

      if (*iter_x < -5.0) {
        *iter_z = -2.0;
      } else if (*iter_x < 0) {
        *iter_z = -1.0;
      } else if (*iter_x < 5) {
        *iter_z = 1.0;
      } else {
        *iter_z = 2.0;
      }
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(builder_node->get_node_base_interface());
  executor.add_node(test_node->get_node_base_interface());

  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    pub->publish(cloud);
    executor.spin_some();
    builder_node->cycle();
    manager->update(navstate);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(navstate.has("map"));
  auto map = navstate.get<grid_map::GridMap>("map");
  ASSERT_TRUE(map.exists("elevation"));
  ASSERT_EQ(map.getLayers().size(), 1);

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    ASSERT_EQ(cloud.width, 10);
    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      grid_map::Position pos(*iter_x, *iter_y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        auto z = map.at("elevation", index);

        if (*iter_x < -5.0) {
          EXPECT_NEAR(z, -2.0, 0.0001);
        } else if (*iter_x < 0) {
          EXPECT_NEAR(z, -1.0, 0.0001);
        } else if (*iter_x < 5) {
          EXPECT_NEAR(z, 1.0, 0.0001);
        } else {
          EXPECT_NEAR(z, 2.0, 0.0001);
        }
      }
    }
  }

  map.add("other_layer", 0.0);
  map.atPosition("other_layer", grid_map::Position(0.0, 0.0)) = 2.0;

  ASSERT_TRUE(map.exists("elevation"));
  ASSERT_TRUE(map.exists("other_layer"));
  ASSERT_EQ(map.getLayers().size(), 2);

  navstate.set("map", map);

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = -10.0 + i * 2.0;
      *iter_y = -10.0 + i * 2.0;

      if (*iter_x < -5.0) {
        *iter_z = 2.0;
      } else if (*iter_x < 0) {
        *iter_z = 1.0;
      } else if (*iter_x < 5) {
        *iter_z = -1.0;
      } else {
        *iter_z = -2.0;
      }
    }
  }

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    pub->publish(cloud);
    executor.spin_some();
    builder_node->cycle();
    manager->update(navstate);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(navstate.has("map"));
  map = navstate.get<grid_map::GridMap>("map");
  ASSERT_TRUE(map.exists("elevation"));
  ASSERT_TRUE(map.exists("other_layer"));
  ASSERT_EQ(map.getLayers().size(), 2);

  ASSERT_NEAR(map.atPosition("other_layer", grid_map::Position(0.0, 0.0)), 2.0, 0.01);

  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    ASSERT_EQ(cloud.width, 10);
    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      grid_map::Position pos(*iter_x, *iter_y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        auto z = map.at("elevation", index);

        if (*iter_x < -5.0) {
          EXPECT_NEAR(z, 2.0, 0.0001);
        } else if (*iter_x < 0) {
          EXPECT_NEAR(z, 1.0, 0.0001);
        } else if (*iter_x < 5) {
          EXPECT_NEAR(z, -1.0, 0.0001);
        } else {
          EXPECT_NEAR(z, -2.0, 0.0001);
        }
      }
    }
  }
}
