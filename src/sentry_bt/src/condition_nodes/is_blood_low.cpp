#include "sentry_bt/condition_nodes/is_blood_low.hpp"

IsBloodLow::IsBloodLow(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
    config.blackboard->get("node", node_);
    config.blackboard->get("blackboard", blackboard);
}

BT::PortsList IsBloodLow::providedPorts()
{
    return {};
}

BT::NodeStatus IsBloodLow::tick()
{
    if (blackboard->isBloodLow()) {
        RCLCPP_INFO(node_->get_logger(), "[IsBloodLow]: 血量过低 (当前: %d, 阈值: %d)",
                    blackboard->getBlood(), blackboard->getBloodThreshold());
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_INFO(node_->get_logger(), "[IsBloodLow]: 血量正常 (当前: %d, 阈值: %d)",
                     blackboard->getBlood(), blackboard->getBloodThreshold());
        return BT::NodeStatus::FAILURE;
    }
}
