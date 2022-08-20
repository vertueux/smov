from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_movement',
            executable='servo_executable',
            name="servo_control",
            output='screen',
        )
    ])