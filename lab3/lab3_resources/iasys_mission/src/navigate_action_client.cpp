#include <cstdlib>
#include <memory>
#include <string>

#include "iasys_interfaces/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavigateActionClient : public rclcpp::Node {
public:
  using NavigateToPose = iasys_interfaces::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigateActionClient(
      double goal_x, double goal_y,
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("navigate_action_client", node_options), goal_x_(goal_x),
        goal_y_(goal_y) {
    client_ptr_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&NavigateActionClient::send_goal, this));
  }

private:
  void send_goal() {
    timer_->cancel();

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(),
                   "Action server not available after waiting for 5 seconds.");

      rclcpp::shutdown();
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.x = goal_x_;
    goal_msg.y = goal_y_;

    RCLCPP_INFO(get_logger(), "Sending navigation goal: x=%.2f, y=%.2f",
                goal_msg.x, goal_msg.y);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&NavigateActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigateActionClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback = std::bind(
        &NavigateActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigate::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal was accepted by the action server.");
  }

  void feedback_callback(
      GoalHandleNavigate::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaining: %.2f m",
                feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigate::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal Succeeded.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Goal was canceled.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code.");
      break;
    }

    if (result.result) {
      RCLCPP_INFO(
          get_logger(), "Result: success=%s, final_distance=%.2f, message='%s'",
          result.result->success ? "true" : "false",
          result.result->final_distance, result.result->message.c_str());
    }
    rclcpp::shutdown();
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  double goal_x_;
  double goal_y_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_action_client"),
                 "Usage: navigate_action_client X Y");
    rclcpp::shutdown();
    return 1;
  }

  const double goal_x = std::stod(argv[1]);
  const double goal_y = std::stod(argv[2]);
  auto node = std::make_shared<NavigateActionClient>(goal_x, goal_y);

  rclcpp::spin(node);

  // TODO 1: create an rclcpp_action client for NavigateToPose on
  // /navigate_to_pose.
  // TODO 2: wait up to 5 seconds for the action server.
  // TODO 3: create a NavigateToPose::Goal and assign goal_x and goal_y.
  // TODO 4: configure goal-response, feedback and result callbacks.
  // TODO 5: send the goal asynchronously and spin the node until the result
  // callback shuts ROS down.

  /* RCLCPP_WARN(
     node->get_logger(),
     "LAB3 starter: complete the TODOs to send goal (%.2f, %.2f).", goal_x,
     goal_y);
   */

  rclcpp::shutdown();
  return 0;
}
