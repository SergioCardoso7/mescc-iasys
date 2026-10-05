#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "iasys_interfaces/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class NavigateActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = iasys_interfaces::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  NavigateActionServer()
  : Node("navigate_action_server")
  {
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.30);
    path_wait_timeout_s_ = declare_parameter<double>("path_wait_timeout_s", 2.0);
    mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 60.0);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    stop_path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&NavigateActionServer::onOdom, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&NavigateActionServer::onPath, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "/navigate_to_pose",
      std::bind(&NavigateActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigateActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&NavigateActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NavigateToPose action server ready.");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    have_odom_ = true;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (waiting_for_new_path_ && !msg->poses.empty()) {
      path_seen_ = true;
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (!std::isfinite(goal->x) || !std::isfinite(goal->y)) {
      RCLCPP_WARN(get_logger(), "Rejecting non-finite goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_active_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal because another mission is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Accepted goal request x=%.2f y=%.2f", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigate>)
  {
    RCLCPP_INFO(get_logger(), "Cancel request accepted.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&NavigateActionServer::execute, this, goal_handle)}.detach();
  }

  void publishGoal(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "odom";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
  }

  void stopVehicle()
  {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "odom";
    stop_path_pub_->publish(empty_path);
  }

  bool currentPosition(double & x, double & y)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_odom_) {
      return false;
    }
    x = current_x_;
    y = current_y_;
    return true;
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    path_seen_ = false;
    waiting_for_new_path_ = true;
    publishGoal(goal->x, goal->y);

    const auto start_time = std::chrono::steady_clock::now();
    const auto path_deadline = start_time + std::chrono::duration<double>(path_wait_timeout_s_);
    rclcpp::Rate rate(5.0);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < path_deadline) {
      if (goal_handle->is_canceling()) {
        stopVehicle();
        result->success = false;
        result->final_distance = -1.0;
        result->message = "Mission canceled before planning completed.";
        goal_handle->canceled(result);
        waiting_for_new_path_ = false;
        goal_active_.store(false);
        return;
      }
      if (path_seen_.load()) {
        break;
      }
      rate.sleep();
    }

    waiting_for_new_path_ = false;
    if (!path_seen_.load()) {
      stopVehicle();
      result->success = false;
      result->final_distance = -1.0;
      result->message = "Planner did not publish a valid path.";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_active_.store(false);
      return;
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        double x = 0.0;
        double y = 0.0;
        currentPosition(x, y);
        const double distance = std::hypot(goal->x - x, goal->y - y);
        stopVehicle();
        result->success = false;
        result->final_distance = distance;
        result->message = "Mission canceled by client.";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Mission canceled with %.2f m remaining.", distance);
        goal_active_.store(false);
        return;
      }

      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >
        mission_timeout_s_)
      {
        stopVehicle();
        result->success = false;
        result->final_distance = -1.0;
        result->message = "Mission timed out.";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "Mission timed out.");
        goal_active_.store(false);
        return;
      }

      double x = 0.0;
      double y = 0.0;
      if (!currentPosition(x, y)) {
        rate.sleep();
        continue;
      }

      const double distance = std::hypot(goal->x - x, goal->y - y);
      feedback->distance_remaining = distance;
      goal_handle->publish_feedback(feedback);

      if (distance <= goal_tolerance_) {
        result->success = true;
        result->final_distance = distance;
        result->message = "Goal reached.";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal reached with final error %.2f m.", distance);
        goal_active_.store(false);
        return;
      }

      rate.sleep();
    }

    stopVehicle();
    result->success = false;
    result->final_distance = -1.0;
    result->message = "ROS shutdown interrupted the mission.";
    goal_handle->abort(result);
    goal_active_.store(false);
  }

  double goal_tolerance_{0.30};
  double path_wait_timeout_s_{2.0};
  double mission_timeout_s_{60.0};

  std::mutex data_mutex_;
  double current_x_{0.0};
  double current_y_{0.0};
  bool have_odom_{false};
  std::atomic_bool goal_active_{false};
  std::atomic_bool path_seen_{false};
  std::atomic_bool waiting_for_new_path_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr stop_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateActionServer>());
  rclcpp::shutdown();
  return 0;
}
