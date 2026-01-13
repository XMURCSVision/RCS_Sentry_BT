#include "sentry_bt/ros/ros.hpp"

Ros::Ros(rclcpp::Node::SharedPtr node_, BlackboardPtr blackboard_) {
    blackboard = blackboard_;
    RCLCPP_INFO(node_->get_logger(), "[Ros]: 创建血量订阅者");
    blood_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/sentry_bt/sentry_blood", 10,
        [this, node_](const std_msgs::msg::Int32::SharedPtr msg) {
          this->blackboard->updateBlood(msg->data, node_);
        });
    RCLCPP_INFO(node_->get_logger(), "[Ros]: 创建姿态发布者");
    mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/sentry_bt/mode", 10);
}

void Ros::publish() {
    std_msgs::msg::Int32 mode;
    mode.data = blackboard->getMode();
    mode_pub_->publish(mode);
}