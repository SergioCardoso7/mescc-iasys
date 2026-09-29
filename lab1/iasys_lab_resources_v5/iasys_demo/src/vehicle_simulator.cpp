#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class VehicleSimulator : public rclcpp::Node
{
public:
  VehicleSimulator()
  : Node("vehicle_simulator")
  {
    start_x_ = declare_parameter<double>("start_x", -7.0);
    start_y_ = declare_parameter<double>("start_y", -6.0);
    start_yaw_ = declare_parameter<double>("start_yaw", 0.0);
    x_ = start_x_;
    y_ = start_y_;
    yaw_ = start_yaw_;
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 1.2);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.8);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&VehicleSimulator::onCommand, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Kept for backwards compatibility with the first classroom version.
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/vehicle_marker", 10);
    vehicle_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/vehicle_markers", 10);
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/reset_vehicle",
      std::bind(
        &VehicleSimulator::onReset, this,
        std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&VehicleSimulator::update, this));
  }

private:
  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static visualization_msgs::msg::Marker makeVehiclePart(
    const rclcpp::Time & stamp, int id, int type,
    double x, double y, double z,
    double sx, double sy, double sz,
    float r, float g, float b, float a = 1.0F)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "base_link";
    marker.ns = "vehicle_model";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.frame_locked = true;
    return marker;
  }

  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_cmd_ = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
    w_cmd_ = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
  }


  void onReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    x_ = start_x_;
    y_ = start_y_;
    yaw_ = start_yaw_;
    v_cmd_ = 0.0;
    w_cmd_ = 0.0;
    trajectory_.poses.clear();
    last_time_ = now();
    response->success = true;
    response->message = "Vehicle reset to the initial pose.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void publishVehicleMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Chassis and cabin.
    markers.markers.push_back(makeVehiclePart(
      stamp, 0, visualization_msgs::msg::Marker::CUBE,
      0.0, 0.0, 0.16, 0.95, 0.48, 0.22,
      0.08F, 0.32F, 0.88F));
    markers.markers.push_back(makeVehiclePart(
      stamp, 1, visualization_msgs::msg::Marker::CUBE,
      -0.08, 0.0, 0.32, 0.42, 0.40, 0.18,
      0.20F, 0.55F, 0.98F));

    // Four wheels.
    int wheel_id = 10;
    for (const double wx : {-0.30, 0.30}) {
      for (const double wy : {-0.28, 0.28}) {
        markers.markers.push_back(makeVehiclePart(
          stamp, wheel_id++, visualization_msgs::msg::Marker::CUBE,
          wx, wy, 0.10, 0.25, 0.09, 0.12,
          0.06F, 0.06F, 0.08F));
      }
    }

    // Heading arrow in the local +X direction.
    markers.markers.push_back(makeVehiclePart(
      stamp, 20, visualization_msgs::msg::Marker::ARROW,
      0.48, 0.0, 0.22, 0.48, 0.12, 0.12,
      0.98F, 0.65F, 0.08F));

    // Vehicle label.
    auto label = makeVehiclePart(
      stamp, 30, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0.0, 0.0, 0.72, 0.0, 0.0, 0.32,
      0.08F, 0.12F, 0.18F);
    label.text = "VEHICLE";
    markers.markers.push_back(label);

    vehicle_markers_pub_->publish(markers);
  }

  void publishTrajectory(const rclcpp::Time & stamp)
  {
    constexpr double kMinSampleDistance = 0.08;
    bool add_sample = trajectory_.poses.empty();
    if (!add_sample) {
      const auto & last = trajectory_.poses.back().pose.position;
      add_sample = std::hypot(x_ - last.x, y_ - last.y) >= kMinSampleDistance;
    }

    if (add_sample) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "odom";
      pose.pose.position.x = x_;
      pose.pose.position.y = y_;
      pose.pose.orientation.w = 1.0;
      trajectory_.poses.push_back(pose);
      if (trajectory_.poses.size() > 2500U) {
        trajectory_.poses.erase(trajectory_.poses.begin(), trajectory_.poses.begin() + 500);
      }
    }

    trajectory_.header.stamp = stamp;
    trajectory_.header.frame_id = "odom";
    trajectory_pub_->publish(trajectory_);
  }

  void update()
  {
    const auto current_time = now();
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0 || dt > 0.2) {
      return;
    }

    x_ += v_cmd_ * std::cos(yaw_) * dt;
    y_ += v_cmd_ * std::sin(yaw_) * dt;
    yaw_ = wrapAngle(yaw_ + w_cmd_ * dt);

    const double half_yaw = yaw_ / 2.0;
    const double qz = std::sin(half_yaw);
    const double qw = std::cos(half_yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = v_cmd_;
    odom.twist.twist.angular.z = w_cmd_;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(transform);

    // Original simple marker retained for compatibility/debugging.
    visualization_msgs::msg::Marker marker;
    marker.header = odom.header;
    marker.ns = "vehicle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = odom.pose.pose;
    marker.pose.position.z = 0.18;
    marker.scale.x = 0.9;
    marker.scale.y = 0.42;
    marker.scale.z = 0.28;
    marker.color.r = 0.10F;
    marker.color.g = 0.35F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker_pub_->publish(marker);

    publishVehicleMarkers(current_time);
    publishTrajectory(current_time);
  }

  double start_x_{-7.0};
  double start_y_{-6.0};
  double start_yaw_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double v_cmd_{0.0};
  double w_cmd_{0.0};
  double max_linear_speed_{1.2};
  double max_angular_speed_{1.8};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path trajectory_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleSimulator>());
  rclcpp::shutdown();
  return 0;
}
