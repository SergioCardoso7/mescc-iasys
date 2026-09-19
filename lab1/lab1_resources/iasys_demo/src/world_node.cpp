#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class WorldNode : public rclcpp::Node
{
public:
  WorldNode()
  : Node("world"),
    resolution_(0.20), width_(100), height_(100), origin_x_(-10.0), origin_y_(-10.0),
    obstacles_{{
      {-2.5, -2.0, -1.0, 4.5},
      {1.0, -5.0, 2.2, 1.5},
      {3.5, 2.0, 6.0, 3.0},
      {-7.0, 2.5, -4.5, 3.5}}}
  {
    start_x_ = declare_parameter<double>("start_x", -7.0);
    start_y_ = declare_parameter<double>("start_y", -6.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/world_markers", qos);
    timer_ = create_wall_timer(1s, std::bind(&WorldNode::publishWorld, this));
    publishWorld();
  }

private:
  struct Obstacle
  {
    double xmin;
    double ymin;
    double xmax;
    double ymax;
  };

  void setOccupied(std::vector<int8_t> & data, double x, double y) const
  {
    const int gx = static_cast<int>((x - origin_x_) / resolution_);
    const int gy = static_cast<int>((y - origin_y_) / resolution_);
    if (gx >= 0 && gy >= 0 && gx < width_ && gy < height_) {
      data[static_cast<std::size_t>(gy * width_ + gx)] = 100;
    }
  }

  visualization_msgs::msg::Marker makeText(
    const rclcpp::Time & stamp, int id, const std::string & text,
    double x, double y, double z, double size) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "odom";
    marker.ns = "labels";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = size;
    marker.color.r = 0.10F;
    marker.color.g = 0.13F;
    marker.color.b = 0.18F;
    marker.color.a = 1.0F;
    marker.text = text;
    return marker;
  }

  void publishWorld()
  {
    const auto stamp = now();
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = "odom";
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(width_ * height_), 0);

    for (int gx = 0; gx < width_; ++gx) {
      grid.data[static_cast<std::size_t>(gx)] = 100;
      grid.data[static_cast<std::size_t>((height_ - 1) * width_ + gx)] = 100;
    }
    for (int gy = 0; gy < height_; ++gy) {
      grid.data[static_cast<std::size_t>(gy * width_)] = 100;
      grid.data[static_cast<std::size_t>(gy * width_ + width_ - 1)] = 100;
    }

    for (const auto & obstacle : obstacles_) {
      for (double x = obstacle.xmin; x <= obstacle.xmax; x += resolution_ / 2.0) {
        for (double y = obstacle.ymin; y <= obstacle.ymax; y += resolution_ / 2.0) {
          setOccupied(grid.data, x, y);
        }
      }
    }
    map_pub_->publish(grid);

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto & obstacle : obstacles_) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "odom";
      marker.ns = "obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = (obstacle.xmin + obstacle.xmax) / 2.0;
      marker.pose.position.y = (obstacle.ymin + obstacle.ymax) / 2.0;
      marker.pose.position.z = 0.28;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle.xmax - obstacle.xmin;
      marker.scale.y = obstacle.ymax - obstacle.ymin;
      marker.scale.z = 0.56;
      marker.color.r = 0.34F;
      marker.color.g = 0.37F;
      marker.color.b = 0.42F;
      marker.color.a = 1.0F;
      markers.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker boundary;
    boundary.header.stamp = stamp;
    boundary.header.frame_id = "odom";
    boundary.ns = "boundary";
    boundary.id = 100;
    boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    boundary.action = visualization_msgs::msg::Marker::ADD;
    boundary.scale.x = 0.10;
    boundary.color.r = 0.16F;
    boundary.color.g = 0.18F;
    boundary.color.b = 0.22F;
    boundary.color.a = 1.0F;
    for (const auto & xy : std::array<std::pair<double, double>, 5>{{
        {-9.8, -9.8}, {9.8, -9.8}, {9.8, 9.8}, {-9.8, 9.8}, {-9.8, -9.8}}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      p.z = 0.03;
      boundary.points.push_back(p);
    }
    markers.markers.push_back(boundary);

    // Start pad and label make the mission immediately readable to students.
    visualization_msgs::msg::Marker start;
    start.header.stamp = stamp;
    start.header.frame_id = "odom";
    start.ns = "mission";
    start.id = 200;
    start.type = visualization_msgs::msg::Marker::CYLINDER;
    start.action = visualization_msgs::msg::Marker::ADD;
    start.pose.position.x = start_x_;
    start.pose.position.y = start_y_;
    start.pose.position.z = 0.035;
    start.pose.orientation.w = 1.0;
    start.scale.x = 0.75;
    start.scale.y = 0.75;
    start.scale.z = 0.07;
    start.color.r = 0.16F;
    start.color.g = 0.48F;
    start.color.b = 0.92F;
    start.color.a = 0.75F;
    markers.markers.push_back(start);
    markers.markers.push_back(makeText(stamp, 201, "START", start_x_, start_y_ - 0.70, 0.18, 0.34));

    markers.markers.push_back(makeText(
      stamp, 300, "IASYS MINI AUTONOMOUS SYSTEM", 0.0, 9.25, 0.18, 0.42));

    marker_pub_->publish(markers);
  }

  const double resolution_;
  const int width_;
  const int height_;
  const double origin_x_;
  const double origin_y_;
  const std::array<Obstacle, 4> obstacles_;
  double start_x_{-7.0};
  double start_y_{-6.0};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldNode>());
  rclcpp::shutdown();
  return 0;
}
