1. install gazebo

2. in terminal enter:
	roslaunch urdf02_gazebo demo01_helloworld.launch 
	roslaunch urdf02_gazebo demo02_car.launch
	roslaunch urdf02_gazebo demo03_env.launch
		=>  move control use:  	
			rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.3 _turn:=0.5
		=>  show movement in rviz:
			roslaunch urdf02_gazebo demo04_sensor.launch 
			in rviz: set "Fixed Frame" as "odom", add "Odometry", set Topic of "Odometry" as "/odom"
			Then you can see the movement of the car not only in gazebo but also in rviz
		=>  show laser cloud points in rviz:
			roslaunch urdf02_gazebo demo04_sensor.launch 
			in rviz: set "Fixed Frame" as "odom", add "LaserScan", set Topic of "LaserScan" as "/scan"
			Then you can see the laser cloud point not only in gazebo but also in rviz		
		=>  show camera images in rviz:
			roslaunch urdf02_gazebo demo04_sensor.launch 
			in rviz: set "Fixed Frame" as "odom", add "Camera", set Topic of "Camera" as "/camera/image_raw"
			Then you can see the camera images in rviz
		=>  show depth images (kinect) in rviz:
			roslaunch urdf02_gazebo demo04_sensor.launch 
			in rviz: set "Fixed Frame" as "odom", add "Camera", set Topic of "Camera" as "/camera/depth/image_raw" or "/camera/rgb/image_raw"
			Then you can see the kinect images in rviz
		=>  show pointcloud2 of kinect in rviz:
			roslaunch urdf02_gazebo demo04_sensor.launch 
			in rviz: set "Fixed Frame" as "odom", add "PointCloud2", set Topic of "PointCloud2" as "/camera/depth/points"
			Then you can see the pointcloud2 in rviz

3. result see:
	show_laser_in_gazebo.png
	show_laser_camera_kinect_in_rviz.png
