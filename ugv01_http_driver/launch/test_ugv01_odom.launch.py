#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Package names
    description_pkg = 'ugv01_description'		# URDF
    rviz_pkg = 'ugv01_http_driver'				# RViz
    driver_pkg = 'ugv01_http_driver'      		# package that contains UGV01HttpDriver

    # Paths for URDF and RViz config
    description_share = get_package_share_directory(description_pkg)
    rviz_share = get_package_share_directory(rviz_pkg)
    urdf_file = os.path.join(description_share, 'src', 'description', 'ugv01_description.urdf')
    rviz_config_file = os.path.join(rviz_share, 'rviz', 'ugv01_odom.rviz')

    # Read URDF contents for robot_description parameter
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()


    # 1) UGV01 HTTP driver node (publishes /odom, reads encoders, applies /cmd_vel)
    ugv01_driver_node = Node(
        package=driver_pkg,
        executable='ugv01_http_driver',
        name='ugv01_http_driver',
        output='screen',
    )

    # 2) robot_state_publisher with URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3) joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 4) RViz2 with URDF-based config (RobotModel + TF already set)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    return LaunchDescription([
        ugv01_driver_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
