#include "sentry_bt/tactical_state_updater.hpp"

#include <algorithm>
#include <utility>

TacticalStateUpdater::TacticalStateUpdater(rclcpp::Node::SharedPtr node, BlackboardPtr blackboard) : node_(std::move(node)), blackboard_(std::move(blackboard)) {
    if (!node_->has_parameter("enemy_lost_timeout_sec")) {
        node_->declare_parameter("enemy_lost_timeout_sec", 15.0);
    }
}

void TacticalStateUpdater::update(const rclcpp::Time &now) {
    TacticalState &state = blackboard_->getMutableTacticalState();

    if (state.mode_enter_time.nanoseconds() == 0) {
        state.mode_enter_time = now;
        state.subtree_enter_time = now;
        state.last_update_time = now;
        state.mode = TacticalMode::MOVE;
        state.last_mode = TacticalMode::MOVE;
    }

    state.last_update_time = now;

    TacticalMode desired = computeDesiredMode(state, now);
    if (desired != state.mode) {
        blackboard_->switchToMode(desired, now);
    }
    RCLCPP_INFO(node_->get_logger(), "[TacticalStateUpdater]: 当前姿态 %s",
                modeToString(blackboard_->getTacticalState().mode).c_str());
}

TacticalMode TacticalStateUpdater::computeDesiredMode(TacticalState &state,const rclcpp::Time &now) const {
    (void)state;
    const bool blood_low = blackboard_->isBloodLow();
    const bool ammo_low = blackboard_->isAmmoLow();
    if (blood_low || ammo_low) {
        return TacticalMode::DEFENSE;
    }

    const double timeout_sec =
        std::max(0.0, node_->get_parameter("enemy_lost_timeout_sec").as_double());
    const bool enemy_detected = blackboard_->isEnemyDetectedRecently(
        now, rclcpp::Duration::from_seconds(timeout_sec));
    const bool under_attack = blackboard_->isAttacked();
    if (enemy_detected || under_attack) {
        return TacticalMode::ATTACK;
    }

    return TacticalMode::MOVE;
}

std::string TacticalStateUpdater::modeToString(TacticalMode mode) const {
    switch (mode) {
    case TacticalMode::MOVE:
        return "MOVE";
    case TacticalMode::ATTACK:
        return "ATTACK";
    case TacticalMode::DEFENSE:
        return "DEFENSE";
    }

    return "UNKNOWN";
}