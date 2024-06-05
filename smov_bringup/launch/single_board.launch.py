from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('smov_config'),
      'single_board.yaml.example'
    )
    return LaunchDescription([
        Node(
            package='i2c_pwm_board',
            executable='node',
            arguments=['1'],
        ),
        Node(
            package='smov_config',
            executable='servos',
            parameters=[config]
        ),
        Node(
            package='smov_states',
            executable='node',
            parameters=[config],
            prefix="bash -c 'sleep 2.0; $0 $@'"
        ),
        Node(
            package='smov_lcd_panel',
            executable='node',
        )
    ])
