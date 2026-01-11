#include "sentry_bt/condition_nodes/mode_node.hpp"

ModeNode::ModeNode(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
}

BT::PortsList ModeNode::providedPorts() {
    return { BT::InputPort<int>("expected_mode", "期望的模式值 (0=移动, 1=攻击, 2=防御)") };
}

BT::NodeStatus ModeNode::tick() {
    auto expected_mode = getInput<int>("expected_mode");
    if (!expected_mode) {
        RCLCPP_ERROR(node_->get_logger(), "[ModeNode]: 未设置expected_mode参数");
        return BT::NodeStatus::FAILURE;
    }

    int current_mode = blackboard->getMode();

    if (current_mode == expected_mode.value()) {
        RCLCPP_DEBUG(node_->get_logger(), "[ModeNode]: 模式匹配 (当前: %d, 期望: %d)",
                     current_mode, expected_mode.value());
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "[ModeNode]: 模式不匹配 (当前: %d, 期望: %d)",
                     current_mode, expected_mode.value());
        return BT::NodeStatus::FAILURE;
    }
}
