#include "sentry_bt/action_nodes/attack_mode_selector.hpp"

namespace sentry_bt {

AttackModeSelector::AttackModeSelector(const std::string &name,
                                       const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {
  config.blackboard->get("node", node_);
  config.blackboard->get("blackboard", blackboard_);
}

BT::PortsList AttackModeSelector::providedPorts() { return {}; }

BT::NodeStatus AttackModeSelector::tick() {
  AttackMode mode = AttackMode::DIRECT;
  if (blackboard_->isAttacked()) {
    mode = AttackMode::SUPPRESS;
  }

  blackboard_->setAttackMode(mode);
  return BT::NodeStatus::SUCCESS;
}

} // namespace sentry_bt
