#!/usr/bin/env python3
# ROS 2 (Humble) launch for RViz2 with the SCAN-Planner config.
# Ported from the ROS 1 rviz.launch.
#
# NOTE: default.rviz was authored for ROS 1 `rviz`. Some displays/topics may not
# load cleanly under `rviz2` and the config may need regeneration in Phase B.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('scan_planner'), 'launch', 'default.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
