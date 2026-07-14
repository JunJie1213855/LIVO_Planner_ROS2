#!/usr/bin/env python3
# ROS 2 (Humble) launch for the SCAN-Planner planner node + controllers.
# Ported from the ROS 1 run.launch + advanced_param.xml.
#
# Phase-A scope: planner node, controllers, gait publisher.
# Phase-B (deferred, see TODO block at the bottom): simulator.xml include and
# robot_state_publisher + go2 URDF (needs the not-yet-migrated go2_description).

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # --- resolve launch args to concrete Python values ---
    is_real_world = LaunchConfiguration('is_real_world').perform(context).lower() in ('true', '1')
    navi_mode = int(LaunchConfiguration('navi_mode').perform(context))
    sensor_type = LaunchConfiguration('sensor_type').perform(context)
    controller_mode = LaunchConfiguration('controller_mode').perform(context)
    max_vel = float(LaunchConfiguration('max_vel').perform(context))
    max_acc = float(LaunchConfiguration('max_acc').perform(context))
    planning_horizon = float(LaunchConfiguration('planning_horizon').perform(context))

    # Phase-B args (simulator + robot description)
    init_x = float(LaunchConfiguration('init_x').perform(context))
    init_y = float(LaunchConfiguration('init_y').perform(context))
    init_z = float(LaunchConfiguration('init_z').perform(context))
    map_size_x = LaunchConfiguration('map_size_x').perform(context)
    map_size_y = LaunchConfiguration('map_size_y').perform(context)
    map_size_z = LaunchConfiguration('map_size_z').perform(context)
    use_gpu = LaunchConfiguration('use_gpu').perform(context)

    # --- derived topics / flags (port of the ROS 1 $(eval ...) expressions) ---
    body_pose_topic = '/LIO/odom_vehicle' if is_real_world else '/quad_0/body_pose'

    if is_real_world:
        sensor_pose_topic = '/LIO/odom_imu'
    elif sensor_type == 'depth':
        sensor_pose_topic = '/quad_0/camera_pose'
    else:
        sensor_pose_topic = '/quad_0/lidar_pose'

    cloud_topic = '/LIO/clouds_lidar' if is_real_world else '/pcl_render_node/cloud'
    cloud_is_world = (not is_real_world)  # real -> false, sim -> true
    depth_topic = ('/camera/aligned_depth_to_color/image_raw'
                   if is_real_world else '/pcl_render_node/depth')

    # depth-camera intrinsics (real vs sim), from advanced_param.xml
    cx = 317.19183349609375 if is_real_world else 321.04638671875
    cy = 256.4806823730469 if is_real_world else 243.44969177246094
    fx = 609.5884399414062 if is_real_world else 387.229248046875
    fy = 609.22021484375 if is_real_world else 387.229248046875

    # --- scan_planner_node parameters (flat slash-style names, matching declare_parameter) ---
    scan_planner_params = {
        'body_pose_topic': body_pose_topic,

        # sensor / map input
        'grid_map/sensor_type': sensor_type,
        'grid_map/cloud_is_world': cloud_is_world,
        'grid_map/need_extrinsic': is_real_world,

        # planning fsm
        'fsm/navi_mode': navi_mode,
        'fsm/thresh_replan': 1.0,
        'fsm/thresh_no_replan': 0.1,
        'fsm/planning_horizon': planning_horizon,
        'fsm/emergency_time_': 1.0,
        'fsm/fail_safe': True,
        'fsm/max_replan_fail_count': 1000,

        # sliding map
        'grid_map/resolution': 0.05,
        'grid_map/sliding_map_size_x': 10.0,
        'grid_map/sliding_map_size_y': 10.0,
        'grid_map/sliding_map_size_z': 5.0,
        'grid_map/map_sliding_thresh': 0.2,
        'grid_map/double_cylinder_radius': 0.25,
        'grid_map/double_cylinder_offset': 0.18,
        'grid_map/obstacles_inflation_z_up': 0.1,
        'grid_map/obstacles_inflation_z_down': 0.4,

        # depth-camera intrinsics
        'grid_map/cx': cx,
        'grid_map/cy': cy,
        'grid_map/fx': fx,
        'grid_map/fy': fy,

        # depth
        'grid_map/depth_filter_maxdist': 3.0,
        'grid_map/depth_filter_mindist': 0.3,
        'grid_map/depth_filter_margin': 1,
        'grid_map/k_depth_scaling_factor': 1000.0,
        'grid_map/skip_pixel': 2,

        # local fusion
        'grid_map/p_hit': 0.85,
        'grid_map/p_miss': 0.30,
        'grid_map/p_min': 0.12,
        'grid_map/p_max': 0.98,
        'grid_map/p_occ': 0.80,
        'grid_map/max_ray_length': 5.0,
        'grid_map/vis_height': 0.3,

        # planner manager
        'manager/max_vel': max_vel,
        'manager/max_acc': max_acc,
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.2,
        'manager/feasibility_tolerance': 0.5,
        'manager/planning_horizon': planning_horizon,

        # trajectory optimization
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 1.0,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.2,
        'optimization/max_vel': max_vel,
        'optimization/vel_tolerance': 1.0,
        'optimization/max_acc': max_acc,
        'optimization/acc_tolerance': 1.0,
    }

    scan_planner_node = Node(
        package='scan_planner',
        executable='scan_planner_node',
        name='scan_planner_node',
        output='screen',
        parameters=[scan_planner_params],
        remappings=[
            ('/grid_map/body_pose', body_pose_topic),
            ('/grid_map/sensor_pose', sensor_pose_topic),
            ('/grid_map/cloud', cloud_topic),
            ('/grid_map/depth', depth_topic),
        ],
    )

    actions = [scan_planner_node]

    # --- controllers (by controller_mode) ---
    if controller_mode == 'open_loop':
        actions.append(Node(
            package='scan_planner',
            executable='open_loop_controller',
            name='open_loop_controller',
            output='screen',
            parameters=[{'body_pose_topic': body_pose_topic}],
        ))
    elif controller_mode == 'closed_loop':
        actions.append(Node(
            package='scan_planner',
            executable='closed_loop_controller',
            name='closed_loop_controller',
            output='screen',
            parameters=[{
                'body_pose_topic': body_pose_topic,
                'time_forward': 0.8,
                'heading_error_threshold': 0.8,
                'kp_pos': 0.8,
                'kp_yaw': 1.5,
                'max_vx': max_vel,
                'max_vy': 0.35,
                'max_vyaw': 1.0,
                'finish_dist': 0.15,
            }],
        ))
        # kinematic sim only in simulation (real world provides its own odom)
        if not is_real_world:
            actions.append(Node(
                package='scan_planner',
                executable='go2_kinematic_sim',
                name='go2_kinematic_sim',
                output='screen',
                parameters=[{
                    'body_pose_topic': body_pose_topic,
                    'init_x': init_x,
                    'init_y': init_y,
                    'init_z': init_z,
                    'max_vx': max_vel,
                    'max_vy': 0.35,
                    'max_vyaw': 1.0,
                }],
            ))

    # --- gait publisher (visualization of leg motion) ---
    actions.append(Node(
        package='scan_planner',
        executable='go2_gait_publisher',
        name='go2_gait_publisher',
        output='screen',
        parameters=[{'body_pose_topic': body_pose_topic}],
    ))

    # =====================================================================
    # Phase B: robot_state_publisher (go2 URDF) + simulator include.
    # =====================================================================

    # robot_state_publisher with the go2 URDF from go2_description share dir
    go2_urdf = os.path.join(get_package_share_directory('go2_description'),
                            'urdf', 'go2_description.urdf')
    with open(go2_urdf, 'r') as f:
        robot_description = f.read()
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='go2_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    ))

    # simulator include (ported simulator.xml -> simulator.launch.py)
    if not is_real_world:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('scan_planner'), 'launch', 'simulator.launch.py')),
            launch_arguments={
                'is_real_world': str(is_real_world).lower(),
                'sensor_type': sensor_type,
                'use_gpu': use_gpu,
                'init_x': str(init_x),
                'init_y': str(init_y),
                'init_z': str(init_z),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
            }.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('is_real_world', default_value='false'),
        # navi_mode: 1 = 2D Nav Goal, 2 = keypoints (multi-floor), 3 = reference path
        DeclareLaunchArgument('navi_mode', default_value='1'),
        DeclareLaunchArgument('sensor_type', default_value='lidar'),
        # controller_mode: open_loop | closed_loop
        DeclareLaunchArgument('controller_mode', default_value='closed_loop'),
        DeclareLaunchArgument('max_vel', default_value='0.75'),
        DeclareLaunchArgument('max_acc', default_value='0.5'),
        DeclareLaunchArgument('planning_horizon', default_value='7.5'),

        # Phase-B: initial robot pose + sim map size + GPU renderer toggle
        DeclareLaunchArgument('init_x', default_value='-19.0'),
        DeclareLaunchArgument('init_y', default_value='1.0'),
        DeclareLaunchArgument('init_z', default_value='0.3'),
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='40.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('use_gpu', default_value='false'),

        OpaqueFunction(function=launch_setup),
    ])
