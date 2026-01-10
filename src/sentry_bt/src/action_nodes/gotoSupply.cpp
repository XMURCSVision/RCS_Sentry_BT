#include "sentry_bt/action_nodes/gotoSupply.hpp"
#include <memory>
#include <chrono>
#include <thread>

GotoSupply::GotoSupply(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), goal_handle_(nullptr){
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList GotoSupply::providedPorts(){
    return {};
}

BT::NodeStatus GotoSupply::tick(){
    auto logger = node_->get_logger();

    if (status() == BT::NodeStatus::IDLE)
    {
        Point supply_point = blackboard->getSupplyPoint();
        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->get_clock()->now();
        goal_msg.pose.pose.position.x = supply_point.x;
        goal_msg.pose.pose.position.y = supply_point.y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(logger, "[GotoSupply]:前往补给点 x:%.2f y:%.2f",
                    goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    if (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(logger, "[GotoSupply]:未连接到navigate_to_pose服务，继续运行用于测试！");
        // 为了测试目的，模拟到达补给点
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟导航时间
        RCLCPP_INFO(logger, "[GotoSupply]:模拟到达补给点！");
        return BT::NodeStatus::SUCCESS;
    }

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
        send_options.goal_response_callback =
            std::bind(&GotoSupply::goalResponseCallback, this, std::placeholders::_1);
        send_options.feedback_callback =
            std::bind(&GotoSupply::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_options.result_callback =
            std::bind(&GotoSupply::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_options);
    }

    return BT::NodeStatus::RUNNING;
}

void GotoSupply::halt()
{
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "[GotoSupply]:补给导航被暂停。");

    if (goal_handle_)
    {
        client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(logger, "[GotoSupply]:已取消补给导航目标！");
        goal_handle_.reset();
    }
}


void GotoSupply::goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle)
{
    auto logger = node_->get_logger();
    if (goal_handle)
    {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(logger, "[GotoSupply]:补给点已被接收。");
    }
    else
    {
        RCLCPP_ERROR(logger, "[GotoSupply]:补给点被拒绝！");
        this->setStatus(BT::NodeStatus::FAILURE);
    }
}

void GotoSupply::feedbackCallback(const GoalHandleNav::SharedPtr &,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(node_->get_logger(), "[GotoSupply]:剩余距离: %.2f m", feedback->distance_remaining);
}


void GotoSupply::resultCallback(const GoalHandleNav::WrappedResult &result)
{
    auto logger = node_->get_logger();

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "[GotoSupply]:已到达补给点！");
        this->setStatus(BT::NodeStatus::SUCCESS);
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "[GotoSupply]:补给导航中止！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger, "[GotoSupply]:补给导航被取消！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    default:
        RCLCPP_ERROR(logger, "[GotoSupply]:出现未知错误！");
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    }

    goal_handle_.reset();
}
