cmake_minimum_required(VERSION 3.8)
project(sentry_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
  include
  include/sentry_bt
)

add_executable(sentry_bt
  src/main.cpp
  src/blackboard.cpp
  src/action_nodes/goto.cpp
  src/action_nodes/patrol.cpp
  src/action_nodes/gotoSupply.cpp
  src/condition_nodes/is_blood_low.cpp
  src/condition_nodes/mode_node.cpp
  src/condition_nodes/is_attacked.cpp
  src/ros/ros.cpp
)

ament_target_dependencies(sentry_bt
  rclcpp
  rclcpp_action
  nav2_msgs
  behaviortree_cpp_v3
  std_msgs
)

target_link_libraries(sentry_bt yaml-cpp)

install(TARGETS sentry_bt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
