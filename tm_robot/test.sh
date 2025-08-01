#!/bin/bash
ros2 topic pub /spd/plane_roi vision_msgs/msg/BoundingBox2D "{center: {position: {x: 650, y: 430}, theta: 0.0}, size_x: 95.0, size_y: 95.0}" --rate 1

