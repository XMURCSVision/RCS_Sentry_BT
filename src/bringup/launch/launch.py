#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sentry_bt_node = Node(
        package='sentry_bt',
        executable='sentry_bt',
        output='screen'
    )
    return LaunchDescription([sentry_bt_node])
