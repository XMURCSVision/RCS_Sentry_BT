#ifndef SENTRY_BT__BLACKBOARD_HPP_
#define SENTRY_BT__BLACKBOARD_HPP_

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

struct Point{
    double x = 0.0;
    double y = 0.0;
};


class Blackboard{
public:
    Blackboard(rclcpp::Node::SharedPtr node);
    ~Blackboard() = default;

    void setDestination(Point destination);
    Point getDestination() const;

    void setBlood(int blood);
    int getBlood() const;

    void setPatrolPoints(const std::vector<Point>& points);
    const std::vector<Point>& getPatrolPoints() const;

    void setSupplyPoint(const Point& point);
    Point getSupplyPoint() const;

    void setBloodThreshold(int threshold);
    int getBloodThreshold() const;
    bool isBloodLow() const;

private:
    Point current_dest_;
    rclcpp::Node::SharedPtr node_;
    int blood = 100;
    std::vector<Point> patrol_points_;
    Point supply_point_;
    int blood_threshold_ = 50;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blood_sub_;
};

using BlackboardPtr = std::shared_ptr<Blackboard>;

#endif // SENTRY_BT__BLACKBOARD_HPP_
