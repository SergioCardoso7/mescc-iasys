#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VehicleMonitor : public rclcpp::Node {
public:
  VehicleMonitor() : Node("vehicle_monitor") {
    warning_distance_ = declare_parameter<double>("warning_distance", 10.0);
    // TODO 1: create a publisher on /monitor/distance_from_start
    // using std_msgs::msg::Float64.
    //
    distance_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/monitor/distance_from_start", 10);

    // TODO 2: create a subscription to /odom using nav_msgs::msg::Odometry.
    // Bind it to odom_callback().
    //
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&VehicleMonitor::odom_callback, this, std::placeholders::_1));

    // TODO 5: create a service server called /reset_statistics using
    // std_srvs::srv::Trigger and bind it to reset_statistics_callback().
    //

    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/reset_statistics",
        std::bind(&VehicleMonitor::reset_statistics_callback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Vehicle monitor started.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    // TODO 3: on the first odometry message, store x and y as the local origin.
    // Remember to set have_origin_ to true.
    if (!have_origin_) {
      start_x_ = x;
      start_y_ = y;
      have_origin_ = true;
    }

    // TODO 4: compute the Euclidean distance from the stored origin and
    // publish it as std_msgs::msg::Float64 on /monitor/distance_from_start.
    //
    const double distance = std::hypot(x - start_x_, y - start_y_);
    std_msgs::msg::Float64 output;
    output.data = distance;
    distance_pub_->publish(output);

    if (distance > warning_distance_) {
      RCLCPP_WARN(get_logger(), "Vehicle is %.2f m from the monitor origin",
                  output.data);
    }
    // Statistics used by the service exercise.
    // Increment odom_count_ for each received odometry message and keep
    // max_distance_ equal to the largest distance observed since the last
    // reset.
    //
    ++odom_count_;
    max_distance_ = std::max(max_distance_, distance);
  }

  void reset_statistics_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    // TODO 6:
    // 1. Save the old counter and maximum distance in local variables.
    // 2. Reset odom_count_ and max_distance_ to zero.
    // 3. Set response->success to true.
    // 4. Return a useful human-readable response->message.
    // 5. Log the reset with RCLCPP_INFO.
    old_odom_count = odom_count_;
    old_max_distance = max_distance_;
    response->success = true;
    response->message =
        "Statistics reset. Odometry messages: " + std::to_string(odom_count_) +
        ", previous max distance: " + std::to_string(max_distance_);
    RCLCPP_INFO(get_logger(), "Received reset statistics request");
    odom_count_ = 0;
    max_distance_ = 0;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  bool have_origin_{false};
  double start_x_{0.0};
  double start_y_{0.0};

  std::size_t odom_count_{0};
  std::size_t old_odom_count{0};
  double max_distance_{0.0};
  double old_max_distance{0.0};

  double warning_distance_{10.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleMonitor>());
  rclcpp::shutdown();
  return 0;
}
