#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "iasys_demo/grid_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class SimplePlanner : public rclcpp::Node
{
public:
  SimplePlanner()
  : Node("simple_planner")
  {
    goal_x_ = declare_parameter<double>("initial_goal_x", 7.0);
    goal_y_ = declare_parameter<double>("initial_goal_y", 6.0);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.35);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos, std::bind(&SimplePlanner::onMap, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SimplePlanner::onOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&SimplePlanner::onGoal, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Original marker retained for compatibility/debugging.
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
    planner_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/planner_markers", 10);
    timer_ = create_wall_timer(200ms, std::bind(&SimplePlanner::planIfReady, this));
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;
    need_replan_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_msg_ = msg;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "odom" && msg->header.frame_id != "map") {
      RCLCPP_WARN(get_logger(), "Goal frame should be odom/map for this demo.");
    }
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    need_replan_ = true;
    RCLCPP_INFO(get_logger(), "New goal: x=%.2f, y=%.2f", goal_x_, goal_y_);
  }

  void planIfReady()
  {
    if (!need_replan_ || !map_msg_ || !odom_msg_) {
      return;
    }

    const auto & info = map_msg_->info;
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    const double resolution = static_cast<double>(info.resolution);
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const double start_x = odom_msg_->pose.pose.position.x;
    const double start_y = odom_msg_->pose.pose.position.y;

    const auto start = iasys_demo::worldToGrid(
      start_x, start_y, origin_x, origin_y, resolution, width, height);
    const auto goal = iasys_demo::worldToGrid(
      goal_x_, goal_y_, origin_x, origin_y, resolution, width, height);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / resolution)));
    const auto inflated = iasys_demo::inflateGrid(map_msg_->data, width, height, radius_cells);
    const auto cells = iasys_demo::simplifyPath(
      iasys_demo::astar(inflated, width, height, start, goal));

    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No path found to requested goal.");
      publishGoalMarker(false);
      publishPlannerMarkers({}, false);
      need_replan_ = false;
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "odom";
    for (const auto & cell : cells) {
      const auto [x, y] = iasys_demo::gridToWorld(
        cell.x, cell.y, origin_x, origin_y, resolution);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path.poses.back().pose.position.x = goal_x_;
    path.poses.back().pose.position.y = goal_y_;
    path_pub_->publish(path);
    publishGoalMarker(true);
    publishPlannerMarkers(path.poses, true);
    RCLCPP_INFO(get_logger(), "Published path with %zu waypoints.", path.poses.size());
    need_replan_ = false;
  }

  void publishGoalMarker(bool valid)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "odom";
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_x_;
    marker.pose.position.y = goal_y_;
    marker.pose.position.z = 0.12;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.24;
    if (valid) {
      marker.color.r = 0.15F;
      marker.color.g = 0.85F;
      marker.color.b = 0.20F;
    } else {
      marker.color.r = 0.90F;
      marker.color.g = 0.10F;
      marker.color.b = 0.10F;
    }
    marker.color.a = 1.0F;
    goal_marker_pub_->publish(marker);
  }

  void publishPlannerMarkers(
    const std::vector<geometry_msgs::msg::PoseStamped> & path_poses, bool valid)
  {
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray markers;

    // Target disc.
    visualization_msgs::msg::Marker target;
    target.header.stamp = stamp;
    target.header.frame_id = "odom";
    target.ns = "planner_goal";
    target.id = 0;
    target.type = visualization_msgs::msg::Marker::CYLINDER;
    target.action = visualization_msgs::msg::Marker::ADD;
    target.pose.position.x = goal_x_;
    target.pose.position.y = goal_y_;
    target.pose.position.z = 0.045;
    target.pose.orientation.w = 1.0;
    target.scale.x = 0.70;
    target.scale.y = 0.70;
    target.scale.z = 0.09;
    target.color.r = valid ? 0.12F : 0.90F;
    target.color.g = valid ? 0.80F : 0.12F;
    target.color.b = valid ? 0.28F : 0.12F;
    target.color.a = 0.85F;
    markers.markers.push_back(target);

    // Ring around the target.
    visualization_msgs::msg::Marker ring;
    ring.header.stamp = stamp;
    ring.header.frame_id = "odom";
    ring.ns = "planner_goal";
    ring.id = 1;
    ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring.action = visualization_msgs::msg::Marker::ADD;
    ring.scale.x = 0.06;
    ring.color.r = valid ? 0.06F : 0.90F;
    ring.color.g = valid ? 0.55F : 0.10F;
    ring.color.b = valid ? 0.16F : 0.10F;
    ring.color.a = 1.0F;
    constexpr int kSegments = 40;
    constexpr double kRadius = 0.52;
    constexpr double kPi = 3.14159265358979323846;
    for (int i = 0; i <= kSegments; ++i) {
      const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSegments);
      geometry_msgs::msg::Point p;
      p.x = goal_x_ + kRadius * std::cos(angle);
      p.y = goal_y_ + kRadius * std::sin(angle);
      p.z = 0.09;
      ring.points.push_back(p);
    }
    markers.markers.push_back(ring);

    visualization_msgs::msg::Marker label;
    label.header.stamp = stamp;
    label.header.frame_id = "odom";
    label.ns = "planner_goal";
    label.id = 2;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = goal_x_;
    label.pose.position.y = goal_y_ + 0.78;
    label.pose.position.z = 0.20;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.34;
    label.color.r = 0.08F;
    label.color.g = 0.12F;
    label.color.b = 0.15F;
    label.color.a = 1.0F;
    label.text = valid ? "GOAL" : "GOAL - NO PATH";
    markers.markers.push_back(label);

    if (!path_poses.empty()) {
      visualization_msgs::msg::Marker waypoints;
      waypoints.header.stamp = stamp;
      waypoints.header.frame_id = "odom";
      waypoints.ns = "planner_path";
      waypoints.id = 10;
      waypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      waypoints.action = visualization_msgs::msg::Marker::ADD;
      waypoints.scale.x = 0.13;
      waypoints.scale.y = 0.13;
      waypoints.scale.z = 0.13;
      waypoints.color.r = 0.12F;
      waypoints.color.g = 0.78F;
      waypoints.color.b = 0.35F;
      waypoints.color.a = 0.90F;
      for (const auto & pose : path_poses) {
        geometry_msgs::msg::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.10;
        waypoints.points.push_back(p);
      }
      markers.markers.push_back(waypoints);
    }

    planner_markers_pub_->publish(markers);
  }

  double goal_x_{7.0};
  double goal_y_{6.0};
  double robot_radius_{0.35};
  bool need_replan_{true};
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planner_markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
