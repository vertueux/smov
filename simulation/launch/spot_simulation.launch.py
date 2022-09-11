import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory('simulation')
    pkg_spot = get_package_share_directory('spot')
    urdf_file = Path(pkg_spot) / 'urdf/model.urdf'

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_simulation, 'config', 'smov.rviz')], # < Not published yet.
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="smov",
        name="smov_rsp",
        output='screen',
        arguments=[str(pkg_spot)])

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_simulation, 'worlds/default_spot.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        gazebo,
        robot_state_publisher,
        rviz
    ])

