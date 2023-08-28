from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smov_board',
            executable='front',
            name='front_board'
        ),
        Node(
            package='smov_board',
            executable='back',
            name='back_board'
        ),
        Node(
            package='smov_states',
            executable='manager',
            name='smov_states',
            parameters=["data/servos_parameters.yaml"]
        )
    ])
