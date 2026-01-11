#include "sentry_bt/action_nodes/patrol.hpp"
#include <memory>
#include <chrono>
#include <random>
#include <thread>

Patrol::Patrol(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), goal_handle_(nullptr), current_point_index_(0), is_navigating_(false) {
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    patrol_points_ = blackboard->getPatrolPoints();
    RCLCPP_INFO(node_->get_logger(), "[Patrol]:已初始化 %zu 个巡逻点", patrol_points_.size());
}



BT::NodeStatus Patrol::tick(){
    auto logger = node_->get_logger();

    // 检查是否处于移动姿态，只有移动姿态才执行巡逻
    if (!blackboard->isMoveMode()){
        if (is_navigating_){
            RCLCPP_WARN(logger, "[Patrol]: 当前模式为 %d，不在移动模式，正在取消当前导航任务！",
                        blackboard->getMode());
        }
        else{
            RCLCPP_WARN(logger, "[Patrol]: 当前模式为 %d，不在移动模式，停止巡逻任务！",
                        blackboard->getMode());
        }
        halt();
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否连接到导航服务
    if (!client_->wait_for_action_server(std::chrono::milliseconds(100))){
        // 测试模式：直接执行巡逻并返回成功
        if (status() == BT::NodeStatus::IDLE){
            sendNextGoal();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::SUCCESS;
    }
    else if (status() == BT::NodeStatus::IDLE && !is_navigating_){
        sendNextGoal();
    }

    return BT::NodeStatus::RUNNING;
}

void Patrol::sendNextGoal(){
    auto logger = node_->get_logger();
    
    if (patrol_points_.empty()){
        RCLCPP_ERROR(logger, "[Patrol]:巡逻点列表为空！");
        this->setStatus(BT::NodeStatus::FAILURE);
        return;
    }
    
    Point current_point = patrol_points_[current_point_index_];
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose.position.x = current_point.x;
    goal_msg.pose.pose.position.y = current_point.y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(logger, "[Patrol]:前往巡逻点 %zu/%zu - x:%.2f y:%.2f",
                current_point_index_ + 1, patrol_points_.size(),
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    if (!client_->wait_for_action_server(std::chrono::seconds(1))){
        RCLCPP_WARN(logger, "[Patrol]:未连接到navigate_to_pose服务，继续运行用于测试！");
        // 为了测试目的，模拟快速巡逻并返回成功
        RCLCPP_INFO(logger, "[Patrol]:模拟快速巡逻点切换");
        // 随机选择下一个巡逻点
        if (patrol_points_.size() > 1) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distrib(0, patrol_points_.size() - 1);

            size_t next_index;
            do {
                next_index = distrib(gen);
            } while (next_index == current_point_index_);

            current_point_index_ = next_index;
            RCLCPP_INFO(logger, "[Patrol]:随机选择下一个巡逻点 %zu", current_point_index_ + 1);
        }
        this->setStatus(BT::NodeStatus::SUCCESS);
        return;
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
    send_options.goal_response_callback =
        std::bind(&Patrol::goalResponseCallback, this, std::placeholders::_1);
    send_options.feedback_callback =
        std::bind(&Patrol::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_options.result_callback =
        std::bind(&Patrol::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_options);
    is_navigating_ = true;
}

void Patrol::halt(){
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "[Patrol]:巡逻被暂停。");

    if (goal_handle_){
        client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(logger, "[Patrol]:已取消当前巡逻目标！");
        goal_handle_.reset(); 
    }
    
    is_navigating_ = false;
}


void Patrol::goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle){
    auto logger = node_->get_logger();
    if (goal_handle){
        goal_handle_ = goal_handle;
        RCLCPP_INFO(logger, "[Patrol]:巡逻点已被接收。");
    }
    else{
        RCLCPP_ERROR(logger, "[Patrol]:巡逻点被拒绝！");
        is_navigating_ = false;
        this->setStatus(BT::NodeStatus::FAILURE);
    }
}

void Patrol::feedbackCallback(const GoalHandleNav::SharedPtr &,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
    RCLCPP_INFO(node_->get_logger(), "[Patrol]:巡逻点 %zu/%zu - 剩余距离: %.2f m", 
                current_point_index_ + 1, patrol_points_.size(), feedback->distance_remaining);
}


void Patrol::resultCallback(const GoalHandleNav::WrappedResult &result){
    auto logger = node_->get_logger();

    switch (result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "[Patrol]:已到达巡逻点 %zu/%zu！",
                    current_point_index_ + 1, patrol_points_.size());

        // 随机选择下一个巡逻点
        if (patrol_points_.size() > 1) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distrib(0, patrol_points_.size() - 1);

            size_t next_index;
            do {
                next_index = distrib(gen);
            } while (next_index == current_point_index_);

            current_point_index_ = next_index;
            RCLCPP_INFO(logger, "[Patrol]:随机选择下一个巡逻点 %zu", current_point_index_ + 1);
        }

        is_navigating_ = false;
        goal_handle_.reset();
        this->setStatus(BT::NodeStatus::SUCCESS);
        break;
        
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "[Patrol]:巡逻中止！");
        is_navigating_ = false;
        goal_handle_.reset();
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
        
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger, "[Patrol]:巡逻被取消！");
        is_navigating_ = false;
        goal_handle_.reset();
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
        
    default:
        RCLCPP_ERROR(logger, "[Patrol]:出现未知错误！");
        is_navigating_ = false;
        goal_handle_.reset();
        this->setStatus(BT::NodeStatus::FAILURE);
        break;
    }
}
