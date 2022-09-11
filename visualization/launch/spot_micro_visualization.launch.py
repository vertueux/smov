import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_spot_micro = get_package_share_directory('spot_micro')
    pkg_visualization = get_package_share_directory('visualization')

    urdf_file = Path(pkg_spot_micro) / 'urdf/model.urdf'

    # Configure the node.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="smov",
        name="smov_rsp",
        output='screen',
        arguments=[str(urdf_file)]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_visualization, 'config', 'spot_micro.rviz')], 
        condition=IfCondition(LaunchConfiguration('rviz'))
    )


    # Run the node
    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])