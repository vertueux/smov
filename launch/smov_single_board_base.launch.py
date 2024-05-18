from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_pwm_board',
            executable='controller',
            arguments=['1'],
        ),
        Node(
            package='smov_setup_servos',
            executable='node',
            parameters=["config/smov_single_board.yaml.example"]
        ),
        Node(
            package='smov_monitor',
            executable='node',
        )
    ])