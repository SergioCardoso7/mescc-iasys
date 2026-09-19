# IASYS Demo (C++ / ROS 2 Jazzy)

A deliberately small ROS 2 autonomous-system demonstrator for the first IASYS laboratory.
Students **run and observe** it in LAB1. ROS 2 concepts, graph inspection and CLI tools are introduced in LAB2.

## Programming-language policy

IASYS uses **C++17 and `rclcpp`** for ROS 2 nodes and autonomous-system algorithms. Python is used only for ROS 2 launch files and auxiliary tooling where appropriate.

## Architecture

- `world_node` — publishes a small occupancy-grid world and obstacle markers.
- `vehicle_simulator` — integrates a 2-D unicycle kinematic model from `/cmd_vel`; publishes `/odom`, TF and a vehicle marker.
- `simple_planner` — accepts RViz `/goal_pose`, plans around obstacles with a deliberately small A* implementation, and publishes `/path`.
- `simple_controller` — follows `/path` and publishes `/cmd_vel`.

The implementation is intentionally simple. Later laboratories can replace these black-box components with student implementations while preserving the interfaces.

## Build on Ubuntu 24.04 / ROS 2 Jazzy

```bash
mkdir -p ~/iasys_ws/src
cd ~/iasys_ws/src
# Copy the iasys_demo directory here.
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch iasys_demo demo.launch.py
```

The vehicle starts near `(-7,-6)` and receives an initial goal near `(7,6)`. In RViz, use **2D Goal Pose** to select another destination.

## Failure experiment

```bash
ros2 launch iasys_demo demo.launch.py controller_enabled:=false
```

The planner still produces a path but the vehicle does not receive motion commands.

## Design note

The fixed frame is `odom`. This is deliberate: LAB1 does not yet teach ROS frames or localization. Those concepts are introduced later.

## Visualisation in v0.3

The default RViz view now includes a richer vehicle model, start/goal labels, A* waypoints and a driven trajectory. The occupancy-grid display is disabled by default to avoid the harmless RViz/OpenGL shader warning seen on some Ubuntu 24.04 virtual machines; the `/map` topic is still published and used by the planner, and the display can be enabled manually in RViz.
