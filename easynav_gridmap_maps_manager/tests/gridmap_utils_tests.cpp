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
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_gridmap_maps_manager/utils.hpp"

class GridmapMapsLoadSaveTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
  }

  static void TearDownTestSuite()
  {
  }
};


TEST_F(GridmapMapsLoadSaveTest, test_configure_success)
{
  grid_map::GridMap map({"elevation", "occupancy"});
  map.setFrameId("map");

  map.setGeometry(
    grid_map::Length(10.0, 10.0),
    1.0,
    grid_map::Position(1.0, 1.0));

  map["elevation"].setConstant(0.0);
  map["occupancy"].setConstant(0.0);

  for (double x = -4.0; x < 6.1; x = x + 1.0) {
    for (double y = -4.0; y < 6.1; y = y + 1.0) {
      float elevation = y > 0.0 ? 0.0 : 1.0;
      float occupancy = x > 0.0 ? 0.0 : 254.0;

      grid_map::Position pos(x, y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        map.at("elevation", index) = elevation;
        map.at("occupancy", index) = occupancy;
      }
    }
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);

    if (position.y() > 0) {
      ASSERT_NEAR(map.at("elevation", *it), 0.0, 0.00001);
    } else {
      ASSERT_NEAR(map.at("elevation", *it), 1.0, 0.00001);
    }

    if (position.x() > 0) {
      ASSERT_NEAR(map.at("occupancy", *it), 0.0, 0.00001);
    } else {
      ASSERT_NEAR(map.at("occupancy", *it), 254.0, 0.00001);
    }
  }

  ASSERT_TRUE(easynav::save_gridmap("/tmp/test_1.yaml", map));

  grid_map::GridMap map2;
  ASSERT_TRUE(easynav::load_gridmap("/tmp/test_1.yaml", map2));

  ASSERT_EQ(map2.getLayers(), std::vector<std::string>({"elevation", "occupancy"}));

  for (grid_map::GridMapIterator it(map2); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map2.getPosition(*it, position);

    if (position.y() > 0) {
      ASSERT_NEAR(map2.at("elevation", *it), 0.0, 0.00001);
    } else {
      ASSERT_NEAR(map2.at("elevation", *it), 1.0, 0.00001);
    }

    if (position.x() > 0) {
      ASSERT_NEAR(map2.at("occupancy", *it), 0.0, 0.00001);
    } else {
      ASSERT_NEAR(map2.at("occupancy", *it), 254.0, 0.00001);
    }
  }

  ASSERT_EQ(map.getLength().x(), map2.getLength().x());
  ASSERT_EQ(map.getLength().y(), map2.getLength().y());
  ASSERT_EQ(map.getPosition().x(), map2.getPosition().x());
  ASSERT_EQ(map.getPosition().y(), map2.getPosition().y());
  ASSERT_EQ(map.getResolution(), map2.getResolution());
}
