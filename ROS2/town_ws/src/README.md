cd town_ws
colcon build    
	or   colcon build --packages-select village_wang    
	or   colcon build --packages-select village_li
ros2 run village_wang wang2_node
ros2 run village_li li4_node

rqt_graph
