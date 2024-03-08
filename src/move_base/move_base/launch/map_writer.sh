#!/bin/bash
rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '/opt/ep_ws/src/navigation/move_base/maps/cartographer/map.pbstream'"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/opt/ep_ws/src/navigation/move_base/maps/cartographer/map.pbstream -resolution=0.05
