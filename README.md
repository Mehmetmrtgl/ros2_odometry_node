# ros2_odometry_node

A ROS 2 node that publishes odometry for wheeled robots. The package currently focuses on mecanum drive support and converts wheel encoder ticks and wheel RPMs into `/odom` messages and a `wheel_positions` diagnostic topic. An Ackermann implementation stub is present for future expansion.

## Features
- ROS 2 C++ node (`ros2_odometry_node`) built with `rclcpp` and `ament_cmake`.
- Mecanum drive implementation that subscribes to encoder and velocity arrays, computes pose/velocity, and publishes `nav_msgs/Odometry`.
- Launch file and parameter YAML for quick configuration.

## Package layout
```
src/ros2_odometry_node/
├── config/odometry_params.yaml   # Default parameters for mecanum kinematics
├── launch/odometry.launch.py     # Launch file running the node with parameters
├── src/                          # Node implementation and drive-specific logic
├── header/                       # Internal headers for core and drive helpers
└── include/ros2_odometry_node/   # Public headers and interfaces
```

## Building
This is a standard ROS 2 ament package. From your workspace root:

```bash
colcon build --packages-select ros2_odometry_node
source install/setup.bash
```

Ensure dependencies in `package.xml` (e.g., `rclcpp`, `nav_msgs`, `tf2_ros`) are available in your ROS 2 installation.

## Running
Launch the node with the provided parameters:

```bash
ros2 launch ros2_odometry_node odometry.launch.py
```

The launch file loads `config/odometry_params.yaml`, which defaults to the mecanum drive type. You can override parameters on the command line, for example:

```bash
ros2 launch ros2_odometry_node odometry.launch.py \
  ros2_odometry_node.drive_type:=MECANUM \
  ros2_odometry_node.ticks_per_rev:=4200.0
```

## Topics (mecanum)
- Subscribes: `encoder1`, `encoder2`, `velocity1`, `velocity2` (`std_msgs/Float32MultiArray`, each with two wheel values).
- Publishes: `/odom` (`nav_msgs/Odometry`), `wheel_positions` (`std_msgs/Float32MultiArray`).

## Parameters
Key parameters (see `config/odometry_params.yaml`):

- `drive_type` (string): Drive model to use. Default: `"MECANUM"`.
- `ticks_per_rev` (double): Encoder ticks per revolution.
- `wheel_radius` (double): Wheel radius in meters.
- `robot_l` (double): Half-length of the robot footprint in meters.
- `robot_w` (double): Half-width of the robot footprint in meters.
- `angle_epsilon` (double): Threshold to treat rotational changes as zero.

You can edit `config/odometry_params.yaml` or override values via the command line when launching.

## Notes
- The Ackermann implementation is currently a placeholder; the mecanum pipeline is the primary supported drive type.
- Publish encoder and wheel velocity arrays in the expected order so the node can compute accurate odometry.
