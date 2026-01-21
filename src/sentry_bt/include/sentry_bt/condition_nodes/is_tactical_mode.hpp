#ifndef SENTRY_BT__CONDITION_NODES__IS_TACTICAL_MODE_HPP_
#define SENTRY_BT__CONDITION_NODES__IS_TACTICAL_MODE_HPP_

#include "sentry_bt/blackboard.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace sentry_bt {

class IsTacticalMode : public BT::ConditionNode {
public:
  IsTacticalMode(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  BlackboardPtr blackboard_;
  TacticalMode expected_{TacticalMode::MOVE};
};

} // namespace sentry_bt

#endif // SENTRY_BT__CONDITION_NODES__IS_TACTICAL_MODE_HPP_
