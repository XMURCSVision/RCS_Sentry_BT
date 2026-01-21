#include "sentry_bt/action_nodes/gotoSupply.hpp"

#include <chrono>
#include <memory>

GotoSupply::GotoSupply(const std::string &name,
                       const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), goal_handle_(nullptr),
      state_(State::IDLE) {
  config.blackboard->get("node", node_);
  config.blackboard->get("blackboard", blackboard);
  client_ =
      rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList GotoSupply::providedPorts() { return {}; }

BT::NodeStatus GotoSupply::tick() {
  // 无导航服务：测试模式直接成功
  if (!client_->wait_for_action_server(std::chrono::milliseconds(100))) {
    if (status() == BT::NodeStatus::IDLE) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 已到补给点后，等待回血到满血
  if (state_ == State::WAITING_FOR_FULL) {
    if (blackboard->isBloodFull()) {
      state_ = State::IDLE;
      goal_handle_.reset();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  // 发送补给点目标
  if (state_ == State::IDLE) {
    Point supply = blackboard->getSupplyPoint();

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose.position.x = supply.x;
    goal_msg.pose.pose.position.y = supply.y;
    goal_msg.pose.pose.orientation.w = 1.0;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
    send_options.goal_response_callback = std::bind(
        &GotoSupply::goalResponseCallback, this, std::placeholders::_1);
    send_options.feedback_callback =
        std::bind(&GotoSupply::feedbackCallback, this, std::placeholders::_1,
                  std::placeholders::_2);
    send_options.result_callback =
        std::bind(&GotoSupply::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_options);
    state_ = State::NAVIGATING;
    return BT::NodeStatus::RUNNING;
  }

  // 导航中
  return BT::NodeStatus::RUNNING;
}

void GotoSupply::halt() {
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  state_ = State::IDLE;
}

void GotoSupply::goalResponseCallback(
    const GoalHandleNav::SharedPtr &goal_handle) {
  if (goal_handle) {
    goal_handle_ = goal_handle;
  } else {
    state_ = State::IDLE;
    goal_handle_.reset();
  }
}

void GotoSupply::feedbackCallback(
    const GoalHandleNav::SharedPtr &,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  (void)feedback;
}

void GotoSupply::resultCallback(const GoalHandleNav::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    state_ = State::WAITING_FOR_FULL;
    break;
  case rclcpp_action::ResultCode::ABORTED:
  case rclcpp_action::ResultCode::CANCELED:
  default:
    state_ = State::IDLE;
    break;
  }
}
