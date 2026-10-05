cmake_minimum_required(VERSION 3.8)
project(iasys_mission)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iasys_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(navigate_action_server src/navigate_action_server.cpp)
ament_target_dependencies(
  navigate_action_server rclcpp rclcpp_action iasys_interfaces geometry_msgs nav_msgs)

add_executable(navigate_action_client src/navigate_action_client.cpp)
ament_target_dependencies(
  navigate_action_client rclcpp rclcpp_action iasys_interfaces)

install(TARGETS
  navigate_action_server
  navigate_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
