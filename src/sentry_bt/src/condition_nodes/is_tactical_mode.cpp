#include "sentry_bt/condition_nodes/is_tactical_mode.hpp"

namespace sentry_bt {

static TacticalMode parseMode(const std::string &s) {
  if (s == "ATTACK")
    return TacticalMode::ATTACK;
  if (s == "DEFENSE")
    return TacticalMode::DEFENSE;
  if (s == "MOVE")
    return TacticalMode::MOVE;
  return TacticalMode::MOVE;
}

IsTacticalMode::IsTacticalMode(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
  config.blackboard->get("node", node_);
  config.blackboard->get("blackboard", blackboard_);
}

BT::PortsList IsTacticalMode::providedPorts() {
  return {BT::InputPort<std::string>("expected_mode", "MOVE/ATTACK/DEFENSE")};
}

BT::NodeStatus IsTacticalMode::tick() {
  auto expected_str = getInput<std::string>("expected_mode");
  if (!expected_str) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[IsTacticalMode]: missing expected_mode");
    return BT::NodeStatus::FAILURE;
  }

  expected_ = parseMode(expected_str.value());
  const TacticalState &s = blackboard_->getTacticalState();

  return (s.mode == expected_) ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
}

} // namespace sentry_bt
