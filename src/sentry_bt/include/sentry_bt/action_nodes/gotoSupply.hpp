#ifndef SENTRY_BT__ACTION_NODES__GOTOSUPPLY_HPP_
#define SENTRY_BT__ACTION_NODES__GOTOSUPPLY_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "blackboard.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GotoSupply : public BT::ActionNodeBase{
    public:
        GotoSupply(const std::string &name, const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
        void halt() override;

    private:
        enum class State { IDLE, NAVIGATING, WAITING_FOR_FULL };
        rclcpp::Node::SharedPtr node_;
        BlackboardPtr blackboard;
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        GoalHandleNav::SharedPtr goal_handle_;
        State state_;
        void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);
        void feedbackCallback(const GoalHandleNav::SharedPtr &,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void resultCallback(const GoalHandleNav::WrappedResult &result);
};


#endif // SENTRY_BT__ACTION_NODES__GOTOSUPPLY_HPP_
