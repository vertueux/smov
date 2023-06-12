Make sure to have set the directions before running this package :
```bash
ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: -1},{servo: 16, center: 333, range: 100, direction: 1}]"
```
