from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_pwm_board',
            executable='node',
            arguments=['1'],
        ),
        Node(
            package='i2c_pwm_board',
            executable='node',
            arguments=['4'], # Can be changed to 2, etc...
        ),
        Node(
            package='smov_config',
            executable='servos',
            parameters=["smov_config/config/dual_board.yaml.example"]
        ),
        Node(
            package='smov_states',
            executable='node',
            parameters=["smov_config/config/dual_board.yaml.example"],
            prefix="bash -c 'sleep 2.0; $0 $@'"
        ),
        Node(
            package='smov_lcd_panel',
            executable='node',
        )
    ])
