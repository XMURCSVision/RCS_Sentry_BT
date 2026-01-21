#ifndef SENTRY_BT__ACTION_NODES__ATTACK_MODE_SELECTOR_HPP_
#define SENTRY_BT__ACTION_NODES__ATTACK_MODE_SELECTOR_HPP_

#include "sentry_bt/blackboard.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace sentry_bt {

class AttackModeSelector : public BT::SyncActionNode {
public:
  AttackModeSelector(const std::string &name,
                     const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  BlackboardPtr blackboard_;
};

} // namespace sentry_bt

#endif // SENTRY_BT__ACTION_NODES__ATTACK_MODE_SELECTOR_HPP_
