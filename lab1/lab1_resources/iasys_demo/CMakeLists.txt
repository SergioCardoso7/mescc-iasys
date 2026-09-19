cmake_minimum_required(VERSION 3.8)
project(iasys_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(world_node src/world_node.cpp)
ament_target_dependencies(world_node rclcpp geometry_msgs nav_msgs visualization_msgs)

add_executable(vehicle_simulator src/vehicle_simulator.cpp)
ament_target_dependencies(vehicle_simulator rclcpp geometry_msgs nav_msgs tf2_ros visualization_msgs)

add_executable(simple_planner src/simple_planner.cpp)
target_include_directories(simple_planner PRIVATE include)
ament_target_dependencies(simple_planner rclcpp geometry_msgs nav_msgs visualization_msgs)

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs nav_msgs)

install(TARGETS
  world_node
  vehicle_simulator
  simple_planner
  simple_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_grid_utils test/test_grid_utils.cpp)
    if(TARGET test_grid_utils)
      target_include_directories(test_grid_utils PRIVATE include)
    endif()
  endif()
endif()

ament_package()
