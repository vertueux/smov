from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='back_i2c_pwm_board',
            executable='back_i2c_pwm_executable',
            name="i2c_pwm_controller",
            output='screen',
            prefix='sudo -E'
        )
    ])
