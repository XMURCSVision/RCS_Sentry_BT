#include "sentry_bt/action_nodes/goto.hpp"
#include <memory>
#include <chrono>

Goto::Goto(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), goal_handle_(nullptr){
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList Goto::providedPorts(){
    return {};
}

BT::NodeStatus Goto::tick(){
    auto logger = node_->get_logger();

    if (status() == BT::NodeStatus::IDLE)
{
        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->get_clock()->now();
        goal_msg.pose.pose.position.x = blackboard->getDestination().x;
        goal_msg.pose.pose.position.y = blackboard->getDestination().y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(logger, "[Goto]:发送目标点 x:%.2f y:%.2f",
                    goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

        if (!client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(logger, "[Goto]:未连接到navigate_to_pose服务！");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
        send_options.goal_response_callback =
            std::bind(&Goto::goalResponseCallback, this, std::placeholders::_1);
        send_options.feedback_callback =
            std::bind(&Goto::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_options.result_callback =
            std::bind(&Goto::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_options);
    }

    return BT::NodeStatus::RUNNING;
}

void Goto::halt()
{
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "[Goto]:导航被暂停。");

    if (goal_handle_)
    {
        client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(logger, "[Goto]:已取消导航目标！");
        goal_handle_.reset(); 
    }
}


void Goto::goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle)
{
    auto logger = node_->get_logger();
    if (goal_handle)
    {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(logger, "[Goto]:目标点已被接收。");
    }
    else
    {
        RCLCPP_ERROR(logger, "[Goto]:目标点被拒绝！");
        this->setStatus(BT::NodeStatus::FAILURE);
    }
}

void Goto::feedbackCallback(const GoalHandleNav::SharedPtr &,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(node_->get_logger(), "[Goto]:剩余距离: %.2f m", feedback->distance_remaining);
}


void Goto::resultCallback(const GoalHandleNav::WrappedResult &result)
{
    auto logger = node_->get_logger();

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "[Goto]:已导航到目标点！");
        this->setStatus(BT::NodeStatus::SUCCESS);
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "[Goto]:导航中止！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger, "[Goto]:导航被取消！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    default:
        RCLCPP_ERROR(logger, "[Goto]:出现未知错误！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    }

    goal_handle_.reset();
}