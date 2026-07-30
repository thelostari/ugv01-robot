import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package and file paths
    pkg_name = 'ugv01_description'
    pkg_share = get_package_share_directory(pkg_name)

    # URDF file
    urdf_file = os.path.join(pkg_share, 'src', 'description', 'ugv01_description.urdf')

    # RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ugv01.rviz')

    # Read URDF into a string
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # joint_state_publisher_gui node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
