#include "sentry_bt/condition_nodes/is_attack_mode.hpp"

namespace sentry_bt {

static AttackMode parseAttackMode(const std::string &s) {
  if (s == "PEEK")
    return AttackMode::PEEK;
  if (s == "SUPPRESS")
    return AttackMode::SUPPRESS;
  return AttackMode::DIRECT;
}

IsAttackMode::IsAttackMode(const std::string &name,
                           const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
  config.blackboard->get("node", node_);
  config.blackboard->get("blackboard", blackboard_);
}

BT::PortsList IsAttackMode::providedPorts() {
  return {BT::InputPort<std::string>("expected_mode", "DIRECT/PEEK/SUPPRESS")};
}

BT::NodeStatus IsAttackMode::tick() {
  auto expected_str = getInput<std::string>("expected_mode");
  if (!expected_str) {
    RCLCPP_ERROR(node_->get_logger(), "[IsAttackMode]: missing expected_mode");
    return BT::NodeStatus::FAILURE;
  }

  expected_ = parseAttackMode(expected_str.value());
  const TacticalState &s = blackboard_->getTacticalState();

  return (s.attack_mode == expected_) ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::FAILURE;
}

} // namespace sentry_bt
