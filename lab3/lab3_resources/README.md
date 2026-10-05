# IASYS LAB3 student starter

Copy `iasys_interfaces` and `iasys_mission` into `~/iasys_ws/src/`, then build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/iasys_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select iasys_interfaces iasys_mission
source install/setup.bash
```

The action server is complete. The action client compiles but contains TODOs to be completed during LAB3.
