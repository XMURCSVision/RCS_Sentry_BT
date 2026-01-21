#include "sentry_bt/ros/ros.hpp"

Ros::Ros(rclcpp::Node::SharedPtr node_, BlackboardPtr blackboard_) {
  blackboard = blackboard_;
  RCLCPP_INFO(node_->get_logger(), "[Ros]: 创建血量订阅者");
  blood_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/sentry_bt/sentry_blood", 10,
      [this, node_](const std_msgs::msg::Int32::SharedPtr msg) {
        this->blackboard->updateBlood(msg->data, node_);
      });

  RCLCPP_INFO(node_->get_logger(), "[Ros]: 创建弹量订阅者");
  ammo_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/sentry_bt/sentry_ammo", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        this->blackboard->setAmmo(msg->data);
      });

  RCLCPP_INFO(node_->get_logger(),
              "[Ros]: 创建敌人发现订阅者 (/sentry_bt/fire)");
  fire_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/sentry_bt/fire", 10,
      [this, node_](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          this->blackboard->markEnemyDetected(node_->get_clock()->now());
        }
      });

  tactical_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
      "/sentry_bt/tactical_mode", 10);
  attack_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
      "/sentry_bt/attack_mode", 10);
}

void Ros::publish() {

  const TacticalState &s = blackboard->getTacticalState();

  std_msgs::msg::Int32 tactical_mode;
  tactical_mode.data = static_cast<int>(s.mode);
  tactical_mode_pub_->publish(tactical_mode);

  std_msgs::msg::Int32 attack_mode;
  attack_mode.data = static_cast<int>(s.attack_mode);
  attack_mode_pub_->publish(attack_mode);
}