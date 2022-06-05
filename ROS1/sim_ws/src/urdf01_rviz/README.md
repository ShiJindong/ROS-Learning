1. install arbotix

	sudo apt-get install ros-ROS version-arbotix

2. in terminal enter:
	
	roslaunch urdf01_rviz demo07_control.launch 
	
        rostopic pub -r 10 /cmd_vel geometry_msgTwist "linear:
            x: 1.0
            y: 0.0
            z: 0.0
        angular:
            x: 0.0
            y: 0.0
            z: 1.0" 

3. Set "Fixed Frame" in rviz as "odom", the movement of the small car will be seen in rviz.

