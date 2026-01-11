#ifndef SENTRY_BT__CONDITION_NODES__MODE_NODE_HPP_
#define SENTRY_BT__CONDITION_NODES__MODE_NODE_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include "sentry_bt/blackboard.hpp"

class ModeNode : public BT::ConditionNode {
    public:
        ModeNode(const std::string &name, const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        BlackboardPtr blackboard;
};

#endif // SENTRY_BT__CONDITION_NODES__MODE_NODE_HPP_
