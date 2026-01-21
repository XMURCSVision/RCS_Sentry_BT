#ifndef SENTRY_BT__CONDITION_NODES__IS_ATTACK_MODE_HPP_
#define SENTRY_BT__CONDITION_NODES__IS_ATTACK_MODE_HPP_

#include "sentry_bt/blackboard.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace sentry_bt {

class IsAttackMode : public BT::ConditionNode {
public:
  IsAttackMode(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  BlackboardPtr blackboard_;
  AttackMode expected_{AttackMode::DIRECT};
};

} // namespace sentry_bt

#endif // SENTRY_BT__CONDITION_NODES__IS_ATTACK_MODE_HPP_
