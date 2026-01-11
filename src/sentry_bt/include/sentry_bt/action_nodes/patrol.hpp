#ifndef SENTRY_BT__ACTION_NODES__PATROL_HPP_
#define SENTRY_BT__ACTION_NODES__PATROL_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "blackboard.hpp"
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Patrol : public BT::ActionNodeBase{
    public:
        Patrol(const std::string &name, const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts(){
        return {};
    }
        BT::NodeStatus tick() override;
        void halt() override;

    private:
        rclcpp::Node::SharedPtr node_;
        BlackboardPtr blackboard;
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        GoalHandleNav::SharedPtr goal_handle_;
        
        // 巡逻点列表
        std::vector<Point> patrol_points_;
        size_t current_point_index_;
        bool is_navigating_;
        
        void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);
        void feedbackCallback(const GoalHandleNav::SharedPtr &,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void resultCallback(const GoalHandleNav::WrappedResult &result);
        void sendNextGoal();
};


#endif // SENTRY_BT__ACTION_NODES__PATROL_HPP_
