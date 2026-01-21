#ifndef SENTRY_BT__BLACKBOARD_HPP_
#define SENTRY_BT__BLACKBOARD_HPP_

#include <deque>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

enum class TacticalMode : int {
  MOVE = 0,
  ATTACK = 1,
  DEFENSE = 2,
};

enum class AttackMode : int {
  DIRECT = 0,
  PEEK = 1,
  SUPPRESS = 2,
};

struct TacticalState {
  TacticalMode mode{TacticalMode::MOVE};
  TacticalMode last_mode{TacticalMode::MOVE};
  AttackMode attack_mode{AttackMode::DIRECT};

  rclcpp::Time emergency_enter_time{0, 0, RCL_ROS_TIME};

  rclcpp::Time mode_enter_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time subtree_enter_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
};

struct Point {
  double x = 0.0;
  double y = 0.0;
};

class Blackboard {
public:
  Blackboard(rclcpp::Node::SharedPtr node);
  ~Blackboard() = default;

  void setDestination(Point destination);
  Point getDestination() const;

  void setBlood(int blood);
  int getBlood() const;

  void setAmmo(int ammo);
  int getAmmo() const;

  void setAmmoThreshold(int threshold);
  int getAmmoThreshold() const;
  bool isAmmoLow() const;

  void setFullAmmoThreshold(int threshold);
  int getFullAmmoThreshold() const;

  void setPatrolPoints(const std::vector<Point> &points);
  const std::vector<Point> &getPatrolPoints() const;

  void setSupplyPoint(const Point &point);
  Point getSupplyPoint() const;

  void setBloodThreshold(int threshold);
  int getBloodThreshold() const;
  bool isBloodLow() const;

  void setFullBloodThreshold(int threshold);
  int getFullBloodThreshold() const;
  bool isBloodFull() const;

  bool isAttacked() const;
  void markEnemyDetected(const rclcpp::Time &when);
  bool isEnemyDetectedRecently(const rclcpp::Time &now,
                               const rclcpp::Duration &window) const;

  const TacticalState &getTacticalState() const;
  TacticalState &getMutableTacticalState();
  void switchToMode(TacticalMode new_mode, const rclcpp::Time &now);
  void setAttackMode(AttackMode mode);
  int getConsecutiveAttackTicks() const;

  void updateBlood(int blood, rclcpp::Node::SharedPtr node_);

private:
  Point current_dest_;
  rclcpp::Node::SharedPtr node_;
  int blood = 100;
  int ammo_ = 0;
  std::vector<Point> patrol_points_;
  Point supply_point_;
  int blood_threshold_ = 50;
  int full_blood_threshold_ = 100;
  int ammo_threshold_ = 0;
  int full_ammo_threshold_ = 100;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blood_sub_;
  std::deque<int> blood_history_;
  int consecutive_attack_ticks_ = 0;

  rclcpp::Time last_enemy_detect_time_{0, 0, RCL_ROS_TIME};

  TacticalState tactical_state_{};

};

using BlackboardPtr = std::shared_ptr<Blackboard>;

#endif // SENTRY_BT__BLACKBOARD_HPP_
