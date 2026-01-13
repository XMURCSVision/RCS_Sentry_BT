#include "sentry_bt/blackboard.hpp"

Blackboard::Blackboard(rclcpp::Node::SharedPtr node) : node_(node) {}

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

bool Blackboard::isAttacked() const { return consecutive_attack_ticks_ >= 2; }

void Blackboard::updateBlood(int blood,rclcpp::Node::SharedPtr node_){
    int old_blood = this->blood;
    this->blood = blood;
    RCLCPP_INFO(node_->get_logger(), "[Blackboard]: 收到血量更新: %d",
                this->blood);

    this->blood_history_.push_back(this->blood);
    if (this->blood_history_.size() > 3) {
        this->blood_history_.pop_front();
    }
    int damage = old_blood - this->blood;
    if (damage > 0) {
        if (damage >= 30) {
        this->consecutive_attack_ticks_ = 2;
        RCLCPP_INFO(node_->get_logger(), "[Blackboard]: 受到伤害: %d", damage);
        } else {
        if (this->blood_history_.size() >= 2) {
            bool consecutive_damage = true;
            for (size_t i = 1; i < this->blood_history_.size(); ++i) {
            if (this->blood_history_[i] >=
                this->blood_history_[i - 1]) {
                consecutive_damage = false;
                break;
            }
            }
            if (consecutive_damage) {
            this->consecutive_attack_ticks_ =
                this->blood_history_.size();
            } else {
            this->consecutive_attack_ticks_ = 0;
            }
        }
        }
    } else {
        this->consecutive_attack_ticks_ = 0;
    }
    if (this->isAttacked() && this->getMode() == 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "[Blackboard]: 受到攻击！血量变化: %d -> %d", old_blood,
                    this->blood);
        this->setMode(1);
    }
} 