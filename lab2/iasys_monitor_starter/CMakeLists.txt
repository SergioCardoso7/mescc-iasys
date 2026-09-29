cmake_minimum_required(VERSION 3.8)
project(iasys_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(vehicle_monitor src/vehicle_monitor.cpp)
ament_target_dependencies(
  vehicle_monitor
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
)

target_compile_features(vehicle_monitor PUBLIC cxx_std_17)

install(TARGETS
  vehicle_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
