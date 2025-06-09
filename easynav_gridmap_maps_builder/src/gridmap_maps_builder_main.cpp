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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_gridmap_maps_builder/GridmapMapsBuilderNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<easynav::GridmapMapsBuilderNode>(rclcpp::NodeOptions());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  auto tf_node = rclcpp::Node::make_shared("tf_node");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(tf_node->get_clock());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure node");
    rclcpp::shutdown();
    return 1;
  }
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate node");
    rclcpp::shutdown();
    return 1;
  }

  tf2_ros::TransformListener tf_listener(*tf_buffer, tf_node, true);
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    exec.spin_some();

    if (node->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      node->cycle();
    }

    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
