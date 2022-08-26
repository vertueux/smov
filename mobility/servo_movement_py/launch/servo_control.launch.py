from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_movement_py',
            executable='servo_movement_py',
            name="servo_control",
            output='screen',
        )
    ])