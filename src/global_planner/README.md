roslaunch global_planner AStar_node.launch

rosrun tf static_transform_publisher 0.0 0.0  0 0 0 0 1 map base_link 100