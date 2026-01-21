#ifndef SENTRY_BT__TACTICAL_STATE_UPDATER_HPP_
#define SENTRY_BT__TACTICAL_STATE_UPDATER_HPP_

#include "sentry_bt/blackboard.hpp"
#include <rclcpp/rclcpp.hpp>

class TacticalStateUpdater {
    public:
        TacticalStateUpdater(rclcpp::Node::SharedPtr node, BlackboardPtr blackboard);
        ~TacticalStateUpdater() = default;

        void update(const rclcpp::Time &now);
        std::string modeToString(TacticalMode mode) const;
    private:
        rclcpp::Node::SharedPtr node_;
        BlackboardPtr blackboard_;

        TacticalMode computeDesiredMode(TacticalState &state,
                                        const rclcpp::Time &now) const;

};

#endif // SENTRY_BT__TACTICAL_STATE_UPDATER_HPP_
