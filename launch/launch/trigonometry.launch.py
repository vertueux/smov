from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trigonometry',
            executable='state',
            name='smov_trigonometry_state',
            parameters=["data/trig_servos_angles.yaml"]
        )
    ])