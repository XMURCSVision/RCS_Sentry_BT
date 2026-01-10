#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <yaml-cpp/yaml.h>

#include "sentry_bt/blackboard.hpp"
#include "sentry_bt/action_nodes/goto.hpp"
#include "sentry_bt/action_nodes/patrol.hpp"
#include "sentry_bt/action_nodes/gotoSupply.hpp"
#include "sentry_bt/condition_nodes/is_blood_low.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("sentry_bt_node");
    RCLCPP_INFO(ros_node->get_logger(), "Sentry BT 节点启动！");
    auto blackboard = std::make_shared<Blackboard>(ros_node);

    // 加载配置文件
    try {
        YAML::Node config = YAML::LoadFile("config/config.yaml");
        if (config["patrol_points"]) {
            std::vector<Point> patrol_points;
            for (const auto& point_node : config["patrol_points"]) {
                Point point;
                point.x = point_node["x"].as<double>();
                point.y = point_node["y"].as<double>();
                patrol_points.push_back(point);
            }
            blackboard->setPatrolPoints(patrol_points);
            RCLCPP_INFO(ros_node->get_logger(), "已加载 %zu 个巡逻点", patrol_points.size());
        }
        if (config["supply_point"]) {
            Point supply_point;
            supply_point.x = config["supply_point"]["x"].as<double>();
            supply_point.y = config["supply_point"]["y"].as<double>();
            blackboard->setSupplyPoint(supply_point);
            RCLCPP_INFO(ros_node->get_logger(), "已设置补给点: x=%.2f y=%.2f", supply_point.x, supply_point.y);
        }
        if (config["blood_threshold"]) {
            int blood_threshold = config["blood_threshold"].as<int>();
            blackboard->setBloodThreshold(blood_threshold);
            RCLCPP_INFO(ros_node->get_logger(), "已设置血量阈值: %d", blood_threshold);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node->get_logger(), "加载配置文件失败: %s", e.what());
    }

    BT::Blackboard::Ptr bt_blackboard = BT::Blackboard::create();
    bt_blackboard->set("node", ros_node);
    bt_blackboard->set("blackboard", blackboard);

    BT::BehaviorTreeFactory factory;
    //注册节点
    factory.registerNodeType<Goto>("Goto");
    factory.registerNodeType<GotoSupply>("GotoSupply");
    factory.registerNodeType<Patrol>("Patrol");
    factory.registerNodeType<IsBloodLow>("IsBloodLow");

    BT::Tree tree;
    tree = factory.createTreeFromFile("config/sentry_bt.xml", bt_blackboard);
    RCLCPP_INFO(ros_node->get_logger(), "行为树加载成功！");


    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(ros_node);
        tree.tickRoot();
        rate.sleep();
    }

    RCLCPP_INFO(ros_node->get_logger(), "Sentry BT 节点退出！");
    rclcpp::shutdown();
    return 0;
}
