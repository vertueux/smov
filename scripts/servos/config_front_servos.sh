ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 3, center: 287, range: 280, direction: -1},{servo: 14, center: 355, range: 300, direction: 1}]"
ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 250, range: 320, direction: -1}]"
ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 400, range: 280, direction: 1}]"
echo "All front servos have been configured."
