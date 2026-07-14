#!/usr/bin/env python3
# ROS 2 (Humble) simulator launch, ported from simulator.xml.
#
# Brings up the simulated world for SCAN-Planner:
#   - map source: mockamap (procedural) OR map_generator/map_pub (PCD file)
#   - odom_visualization (robot mesh / pose / trajectory markers)
#   - local_sensing_node/pcl_render_node (CPU point-cloud sensor renderer)
#
# All nodes are gated on `not is_real_world`. The GPU renderer
# (opengl_render_node) is deferred -- see the TODO Phase B (GPU) block.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    is_real_world = arg('is_real_world').lower() in ('true', '1')
    sensor_type = arg('sensor_type')
    map_size_x = float(arg('map_size_x'))
    map_size_y = float(arg('map_size_y'))
    map_size_z = float(arg('map_size_z'))
    use_pcd_map = arg('use_pcd_map').lower() in ('true', '1')
    pcd_map_file = arg('pcd_map_file')
    use_gpu = arg('use_gpu').lower() in ('true', '1')
    pcd_frame_id = arg('pcd_frame_id')
    pcd_publish_rate = float(arg('pcd_publish_rate'))
    downsample_resolution = float(arg('downsample_resolution'))

    actions = []

    if is_real_world:
        # Real world provides its own map/odom; nothing to simulate.
        return actions

    # --- map source -----------------------------------------------------
    if use_pcd_map:
        # PCD file map publisher
        actions.append(Node(
            package='map_generator',
            executable='map_pub',
            name='map_pub',
            output='screen',
            arguments=[pcd_map_file],
            parameters=[{
                'frame_id': pcd_frame_id,
                'publish_rate': pcd_publish_rate,
                'cloud_topic': '/map_generator/global_cloud',
                'downsample_res': downsample_resolution,
            }],
        ))
    else:
        # Procedural mock map. NOTE: x/y/z_length are INTEGER params in mockamap.
        actions.append(Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[{
                'seed': 127,
                'update_freq': 0.5,
                # box edge length (m)
                'resolution': 0.1,
                # map size (m) -- integer params
                'x_length': int(map_size_x),
                'y_length': int(map_size_y),
                'z_length': int(map_size_z),
                'type': 2,
                # 1: perlin noise parameters
                'complexity': 0.05,
                'fill': 0.12,
                'fractal': 1,
                'attenuation': 0.1,
                # 2: post2d random box map parameters
                'width_min': 0.2,
                'width_max': 0.8,
                'height_min': 2.0,
                'height_max': 2.0,
                'obstacle_number': 500,
                'surface_resolution': 0.05,
            }],
            remappings=[('/mock_map', '/map_generator/global_cloud')],
        ))

    # --- odometry visualization ----------------------------------------
    actions.append(Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
    ))

    # --- local sensing (CPU renderer) ----------------------------------
    if not use_gpu:
        actions.append(Node(
            package='local_sensing_node',
            executable='pcl_render_node',
            name='pcl_render_node',
            output='screen',
            parameters=[{
                'sensor_type': sensor_type,
                'map/x_size': map_size_x,
                'map/y_size': map_size_y,
                'map/z_size': map_size_z,
            }],
            remappings=[
                # migrated node subscribes to the relative topic "global_map"
                # (ROS 1 "~global_map"). Bind it to the global map cloud.
                ('global_map', '/map_generator/global_cloud'),
                # align sensor outputs with the topics run.launch.py expects
                ('cloud', '/pcl_render_node/cloud'),
                ('depth', '/pcl_render_node/depth'),
            ],
        ))

    # =====================================================================
    # TODO Phase B (GPU): opengl_render_node is deferred. Build the GPU
    # renderer with `-DUSE_GPU=ON` and enable this block (use_gpu:=true).
    #
    # if use_gpu:
    #     actions.append(Node(
    #         package='local_sensing_node',
    #         executable='opengl_render_node',
    #         name='pcl_render_node',
    #         output='screen',
    #         arguments=[pcd_map_file],
    #         parameters=[{
    #             'sensor_type': sensor_type,
    #             'map/x_size': map_size_x,
    #             'map/y_size': map_size_y,
    #             'map/z_size': map_size_z,
    #             'use_global_map_topic': (not use_pcd_map),
    #         }],
    #         remappings=[
    #             ('global_map', '/map_generator/global_cloud'),
    #             ('cloud', '/pcl_render_node/cloud'),
    #             ('depth', '/pcl_render_node/depth'),
    #         ],
    #     ))
    # =====================================================================

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('is_real_world', default_value='false'),
        DeclareLaunchArgument('sensor_type', default_value='lidar'),

        # initial robot pose (consumed by go2_kinematic_sim in run.launch.py)
        DeclareLaunchArgument('init_x', default_value='-19.0'),
        DeclareLaunchArgument('init_y', default_value='1.0'),
        DeclareLaunchArgument('init_z', default_value='0.3'),

        # simulated map size (m)
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='40.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),

        # PCD map source
        DeclareLaunchArgument('use_pcd_map', default_value='false'),
        DeclareLaunchArgument('pcd_map_file', default_value=''),
        DeclareLaunchArgument('pcd_frame_id', default_value='world'),
        DeclareLaunchArgument('pcd_publish_rate', default_value='0.2'),
        DeclareLaunchArgument('downsample_resolution', default_value='0.1'),

        # local sensing
        DeclareLaunchArgument('use_gpu', default_value='false'),

        OpaqueFunction(function=launch_setup),
    ])
