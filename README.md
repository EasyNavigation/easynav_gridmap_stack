# easynav_gridmap_stack


This stack is part of the Easy Navigation (EasyNav) project developed by the Intelligent Robotics Lab. It is composed of the **gridmap_maps_builder** package and the **gridmap_maps_manager** package (currently under construction) for handling elevation maps.



## Installation

Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/EasyNavigation/easynav_gridmap_stack.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easynav_gridmap_maps_builder
```

## Usage

Source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```
Run the lifecycle node:
```bash
ros2 run easynav_gridmap_maps_builder gridmap_maps_builder_node
```

## Parameters

| Name                     | Type   | Default Value | Description |
|--------------------------|--------|----------------|-------------|
| `sensor_topic`           | string | `"points"`     | Topic name to subscribe for point cloud data. |
| `downsample_resolution`  | double | `1.0`          | Resolution used for downsampling the point cloud. |
| `perception_default_frame` | string | `"map"`      | Default frame ID for the output grid map. |
