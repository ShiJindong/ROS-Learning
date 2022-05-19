1. install gmapping map-server navigation:
	sudo apt install ros-<ROS version>-gmapping
	sudo apt install ros-<ROS version>-map-server
	sudo apt install ros-<ROS version>-navigation


2. in terminal enter:
	source sim_ws/devel/setup.bash
	roslaunch urdf02_gazebo demo03_env.launch
		=> this launch file will open the simulation in gazebo automatically
	source navigation/devel/setup.bash
	roslaunch nav_demo nav01_slam.launch
		=> this launch file will open the open visualization in rviz automatically
		=> in rviz: set "Fixed Frame" as "map"
		=> add "RobotModel"
		=> add "LaserScan", set Topic of "LaserScan" as "/scan"	
		=> add "TF", select "Frames": "base_footprint", "map", "odom"
		=> add "Map", set Topic of "Map" as "/map"	

3. open keyboard node
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py
		=> you can use keyborad now to control the movement of robot
	
4. save map
	roslaunch nav_demo nav02_map_save.launch
		=> the map will be saved, you will find "nav.pgm" and "nav.yaml" under the folder "nav_demo/map"

5. read map
	shut all node above
	roslaunch nav_demo nav03_map_server.launch 
		=> publish the map
	rviz
		=> open rviz
		=> in rviz, add "Map", set Topic of "Map" as "/map"
			Then you will see the saved map
			you can set the value in the file "nav.yaml" to change the resolution, position, 
			orientation, occupied_thresh, free_thresh of the map.





