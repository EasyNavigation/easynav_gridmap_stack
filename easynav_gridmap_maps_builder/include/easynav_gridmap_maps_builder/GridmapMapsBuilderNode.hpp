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
 * @brief Lifecycle node that subscribes to point cloud sensor data and manages map building.
 *
 * This node handles perception data (point clouds) using multiple MapsBuilder instances
 * to generate outdoor maps. It supports ROS2 lifecycle management with clean startup,
 * activation, deactivation, and cleanup phases. The node also publishes processed maps
 * (e.g., filtered point clouds) for downstream consumption.
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
   * @brief Perform a processing cycle on the perception data and update maps.
   *
   * This method should be called periodically (e.g., in a timer or main loop) to process
   * incoming sensor data, update the internal map representations, and publish outputs.
   */
  void cycle();

private:
  /// Name of the sensor topic to subscribe to (e.g., point clouds).
  std::string sensor_topic_;

  /// Collection of perception data managed by this node.
  Perceptions perceptions_;

  /// Callback group for concurrency management of subscriptions and timers.
  rclcpp::CallbackGroup::SharedPtr cbg_;

  /// Downsampling resolution applied to point cloud data.
  double downsample_resolution_;

  /// Default frame ID used for perception data and published messages.
  std::string perception_default_frame_;

  /// Publisher for processed gridmap map data.
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDERNODE_HPP_
