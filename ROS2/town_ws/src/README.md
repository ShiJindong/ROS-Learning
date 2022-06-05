cd town_ws

colcon build (--packages-select "packagename")
	
ros2 run village_wang wang2_node

ros2 run village_li li4_node

rqt_graph
