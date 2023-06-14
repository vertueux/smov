ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 3, center: 210, range: 210, direction: -1},{servo: 14, center: 355, range: 190, direction: 1}]"
ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 250, range: 320, direction: -1}]"
ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 400, range: 250, direction: 1}]"
echo "All rear servos have been configured."
