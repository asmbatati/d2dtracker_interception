#!/usr/bin/env python3
"""Launch the interception strategy node.

Remaps its generic I/O onto the interceptor/target namespaces. By default it
consumes the target's predicted path; if that topic is silent it falls back to
a constant-velocity prediction from the target odometry.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'interceptor_odom',
            default_value='/interceptor/mavros/local_position/odom'),
        DeclareLaunchArgument(
            'target_odom', default_value='/target/mavros/local_position/odom'),
        DeclareLaunchArgument(
            'target_predicted_path', default_value='/target/predicted_path'),
        DeclareLaunchArgument('vmax', default_value='8.0'),
        DeclareLaunchArgument('amax', default_value='4.0'),
        Node(
            package='d2dtracker_interception',
            executable='interceptor',
            name='interceptor_node',
            output='screen',
            parameters=[{
                'vmax': LaunchConfiguration('vmax'),
                'amax': LaunchConfiguration('amax'),
                'frame_id': 'map',
                'use_sim_time': True,
            }],
            remappings=[
                ('interceptor_odom', LaunchConfiguration('interceptor_odom')),
                ('target_odom', LaunchConfiguration('target_odom')),
                ('target_predicted_path', LaunchConfiguration('target_predicted_path')),
            ],
        ),
    ])
