# IASYS LAB1 resources — C++ edition

Contents:

- `iasys_demo/` — ROS 2 Jazzy `ament_cmake` package used for the LAB1 autonomous-system demonstration.
- `scripts/check_environment.sh` — quick post-installation check. The script sources `/opt/ros/jazzy/setup.bash` automatically before checking ROS 2 commands.

## Language policy

All functional ROS 2 nodes in `iasys_demo` are implemented in **C++17 with `rclcpp`**. Python is retained only for the standard ROS 2 launch file (`demo.launch.py`).

The student-facing LAB guide is distributed separately as a DOCX/PDF by the instructor.
