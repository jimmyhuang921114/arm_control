#!/bin/bash
ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 0, rail_name: 'arm_rail1'}"

for i in {1..100}
do
  echo "Iteration $i: Call rail"
  ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 1, rail_name: 'arm_rail1'}"
  
  echo "Iteration $i: Return bag"
  ros2 service call rail_control tm_rail_interface/srv/RailControl "{opt_code: 2, rail_name: 'arm_rail1'}"
done