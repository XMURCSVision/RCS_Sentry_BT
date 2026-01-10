#ifndef SENTRY_BT__CONDITION_NODES__IS_BLOOD_LOW_HPP_
#define SENTRY_BT__CONDITION_NODES__IS_BLOOD_LOW_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "blackboard.hpp"

class IsBloodLow : public BT::ConditionNode
{
public:
    IsBloodLow(const std::string &name, const BT::NodeConfiguration &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    BlackboardPtr blackboard;
};

#endif // SENTRY_BT__CONDITION_NODES__IS_BLOOD_LOW_HPP_
