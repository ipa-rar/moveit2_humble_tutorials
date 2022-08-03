#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file = get_package_file('ur5e_workcell_support', 'urdf/ur5e_workcell.urdf.xacro')
    urdf = xacro.process_file(xacro_file).toprettyxml(indent='  ')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}]),
        Node(
            name='joint_state_publisher_gui',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'),
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', get_package_file('ur5e_workcell_support', 'launch/workcell.rviz')]),
    ])
