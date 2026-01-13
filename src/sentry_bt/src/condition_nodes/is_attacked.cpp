#include "sentry_bt/condition_nodes/is_attacked.hpp"

IsAttacked::IsAttacked(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config){
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
}

BT::PortsList IsAttacked::providedPorts(){
    return {};
}

BT::NodeStatus IsAttacked::tick(){
    if (blackboard->isAttacked()) {
        RCLCPP_INFO(node_->get_logger(), "[IsAttacked]: 检测到被攻击");
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_INFO(node_->get_logger(), "[IsAttacked]: 未检测到被攻击");
        return BT::NodeStatus::FAILURE;
    }
}
