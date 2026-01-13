#include "sentry_bt/blackboard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class Ros{
    public:
      Ros(rclcpp::Node::SharedPtr node_, BlackboardPtr blackboard_);
      void publish();

    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blood_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
        BlackboardPtr blackboard;
};