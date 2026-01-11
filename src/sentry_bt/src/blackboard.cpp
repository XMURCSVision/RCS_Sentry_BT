#include "sentry_bt/blackboard.hpp"

Blackboard::Blackboard(rclcpp::Node::SharedPtr node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "[Blackboard]: 创建血量订阅者");
    blood_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/sentry_blood", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int old_blood = this->blood;
            this->blood = msg->data;
            RCLCPP_INFO(node_->get_logger(), "[Blackboard]: 收到血量更新: %d",this->blood);
            if (this->blood < old_blood && this->getMode() == 0) {
                RCLCPP_INFO(node_->get_logger(), "[Blackboard]: 受到攻击！血量变化: %d -> %d",old_blood, this->blood);
                this->setMode(1);
            }
        });
}

void Blackboard::setDestination(Point destination) {
    current_dest_ = destination;
}

Point Blackboard::getDestination() const { return current_dest_; }

void Blackboard::setBlood(int blood) { this->blood = blood; }

int Blackboard::getBlood() const { return blood; }

void Blackboard::setPatrolPoints(const std::vector<Point> &points) {
    patrol_points_ = points;
}

const std::vector<Point> &Blackboard::getPatrolPoints() const {
    return patrol_points_;
}

void Blackboard::setSupplyPoint(const Point &point) { supply_point_ = point; }

Point Blackboard::getSupplyPoint() const { return supply_point_; }

void Blackboard::setBloodThreshold(int threshold) {
    blood_threshold_ = threshold;
}

int Blackboard::getBloodThreshold() const { return blood_threshold_; }

bool Blackboard::isBloodLow() const { return blood < blood_threshold_; }

void Blackboard::setMode(int mode) { this->mode = mode; }

int Blackboard::getMode() const { return mode; }

bool Blackboard::isMoveMode() const { return mode == 0; }

bool Blackboard::isAttackMode() const { return mode == 1; }

bool Blackboard::isDefenseMode() const { return mode == 2; }

void Blackboard::setFullBloodThreshold(int threshold) {
    full_blood_threshold_ = threshold;
}

int Blackboard::getFullBloodThreshold() const { return full_blood_threshold_; }

bool Blackboard::isBloodFull() const { return blood >= full_blood_threshold_; }
