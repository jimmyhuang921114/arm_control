#!/bin/bash
ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 0, rail_name: 'arm_rail1'}"
ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 1, rail_name: 'arm_rail1'}"
ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 2, rail_name: 'arm_rail1'}"