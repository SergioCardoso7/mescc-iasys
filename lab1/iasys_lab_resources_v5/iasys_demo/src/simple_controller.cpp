#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SimpleController : public rclcpp::Node
{
public:
  SimpleController()
  : Node("simple_controller")
  {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.9);
    angular_gain_ = declare_parameter<double>("angular_gain", 2.2);
    waypoint_tolerance_ = declare_parameter<double>("waypoint_tolerance", 0.28);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.22);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&SimpleController::onPath, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SimpleController::onOdom, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, std::bind(&SimpleController::control, this));
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    waypoint_index_ = 0;
    RCLCPP_INFO(get_logger(), "Received path with %zu waypoints.", path_.size());
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_msg_ = msg;
  }

  void control()
  {
    // Read the current parameter values so ros2 param set takes effect at runtime.
    get_parameter("linear_speed", linear_speed_);
    get_parameter("angular_gain", angular_gain_);
    get_parameter("waypoint_tolerance", waypoint_tolerance_);
    get_parameter("goal_tolerance", goal_tolerance_);

    geometry_msgs::msg::Twist command;
    if (!odom_msg_ || path_.empty()) {
      cmd_pub_->publish(command);
      return;
    }

    const double x = odom_msg_->pose.pose.position.x;
    const double y = odom_msg_->pose.pose.position.y;
    const double yaw = yawFromQuaternion(odom_msg_->pose.pose.orientation);

    while (waypoint_index_ + 1 < path_.size()) {
      const auto & target = path_[waypoint_index_];
      if (std::hypot(target.first - x, target.second - y) > waypoint_tolerance_) {
        break;
      }
      ++waypoint_index_;
    }

    const auto & target = path_[waypoint_index_];
    const double distance = std::hypot(target.first - x, target.second - y);
    if (waypoint_index_ + 1 == path_.size() && distance < goal_tolerance_) {
      cmd_pub_->publish(command);
      return;
    }

    const double desired_heading = std::atan2(target.second - y, target.first - x);
    const double heading_error = wrapAngle(desired_heading - yaw);
    command.angular.z = std::clamp(angular_gain_ * heading_error, -1.6, 1.6);

    constexpr double kPi = 3.14159265358979323846;
    const double limited_error = std::min(std::abs(heading_error), kPi / 2.0);
    const double heading_factor = std::max(0.15, std::cos(limited_error));
    command.linear.x = std::min(linear_speed_, 1.4 * distance) * heading_factor;
    cmd_pub_->publish(command);
  }

  double linear_speed_{0.9};
  double angular_gain_{2.2};
  double waypoint_tolerance_{0.28};
  double goal_tolerance_{0.22};
  std::size_t waypoint_index_{0};
  std::vector<std::pair<double, double>> path_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleController>());
  rclcpp::shutdown();
  return 0;
}
