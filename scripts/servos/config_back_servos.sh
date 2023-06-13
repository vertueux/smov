ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 333, range: 100, direction: -1},{servo: 15, center: 333, range: 100, direction: -1},{servo: 14, center: 333, range: 100, direction: -1}]"
ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: 1},{servo: 2, center: 333, range: 100, direction: 1},{servo: 3, center: 333, range: 100, direction: 1}]"
echo "All rear servos have been configured."
