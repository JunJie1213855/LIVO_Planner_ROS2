#!/usr/bin/env python3
"""Ego-Planner-2D-ROS2 交互式仿真一键启动。

启动内容：
  - ego_planner 的 motion_plan 节点
  - 一个 map 静态 TF（保证 RViz Fixed Frame=map 可用）
  - RViz2（加载 src/Ego-Planner-2D-ROS2/rviz/ego_planner.rviz）

启动后在 RViz 中（Fixed Frame = map）：
  1. 用 “2D Goal Pose” 点选终点            -> 生成全局路径 /visual_global_path
  2. 用 “Publish Point” / “2D Pose Estimate” 点选加障碍
  3. 规划默认已“武装”(auto_trigger:=true)，设好目标+障碍即实时出 /visual_local_trajectory；
     也可手动：ros2 topic pub /trigger_plan std_msgs/msg/Bool "{data: true}"

运行：
  cd /home/ros/rosws/planner_ws
  source install/setup.bash          # 需先 colcon build --packages-select ego_planner
  ros2 launch launch/ego_planner_sim.launch.py
  # 关闭 rviz：rviz:=false    关闭自动触发：auto_trigger:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 本文件位于 <ws>/launch/，回到工作区根再定位 rviz 配置
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_cfg = os.path.join(ws_root, 'src', 'Ego-Planner-2D-ROS2', 'rviz', 'ego_planner.rviz')

    use_rviz = LaunchConfiguration('rviz')
    auto_trigger = LaunchConfiguration('auto_trigger')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2 并加载 ego_planner.rviz'),
        DeclareLaunchArgument('auto_trigger', default_value='true',
                              description='启动数秒后自动发布 /trigger_plan true 武装规划器'),

        # 规划主节点
        Node(
            package='ego_planner',
            executable='motion_plan',
            name='ego_planner_interactive_node',
            output='screen',
        ),

        # 保证 RViz 的 Fixed Frame 'map' 存在（ego_planner 自身不广播 TF）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ego_map_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'ego_base'],
            output='log',
        ),

        # 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            condition=IfCondition(use_rviz),
        ),

        # 可选：启动 3 秒后武装规划器（设好目标+障碍即实时规划）
        TimerAction(
            period=3.0,
            condition=IfCondition(auto_trigger),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '-1', '/trigger_plan',
                         'std_msgs/msg/Bool', '{data: true}'],
                    output='log',
                ),
            ],
        ),
    ])
