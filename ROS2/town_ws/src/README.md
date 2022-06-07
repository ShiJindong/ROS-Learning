	cd town_ws
最好先一起编译一次，因为在package.xml中定义了依赖关系，可以保证各个节点编译的先后顺序是正确的:

	colcon build
之后可以选择分别进行编译:

	colcon build --packages-select village_interfaces
	colcon build --packages-select village_li 
	colcon build --packages-select village_wang
	colcon build --packages-select village_zhang
source:

	source install/setup.bash
	
角色说明: 

	li4 = 作家，灵魂写手
	li3 = 作家弟弟，不学无术，好吃懒做，经常借哥哥li4的钱
	wang2 = 单身狗，爱买li4写的小说书来看
	zhang3 = 穷光蛋，爱买wang2的二手书来看
	
功能包village_interface内定义了消息和服务的接口:

	Novel.msg
	BorrowMoney.srv
	SellNovel.srv	
	
节点li4_node:

	他作为消息发布者，会定期发布消息/sexy_girl，来发布自己写的小说
	他也作为消息接收者，接收卖出小说得到的稿费/sexy_girl_money
	同时他也作为服务端，提供了借别人钱的服务/borrow_money，得到请求后发送响应，即借出钱
		ros2 run village_li li4_node

节点wang2_node:

	他作为消息接收者，一旦接收到li4发布的消息/sexy_girl就会将其写的小说买入
	他也作为消息发布者，买入小说后，会给予稿费/sexy_girl_money
	同时他也作为服务端，提供卖二手书的服务/sell_novel，得到请求后发送响应，即卖出二手书
		ros2 run village_wang wang2_node


节点li3_node:

	他作为客户端，会发送借钱请求/borrow_money，然后会得到响应，即得到借的钱
		ros2 run village_li li3_node
		或者手动调用服务:
			ros2 service call /borrow_money village_interfaces/srv/BorrowMoney "{name: 'li3', money: 5}"
			或者使用 rqt, 选择 /borrow_money 服务，填入name和money，然后call

节点zhang3_node:

	他作为客户端，会发送买二手书的请求/sell_novel，然后会得到响应,即买到的二手书
		ros2 run village_zhang zhang3_node
		或者手动调用服务:
			ros2 service call /sell_novel village_interfaces/srv/SellNovel "{money: 5}"
			或者使用 rqt, 选择 /sell_novel 服务，填入money，然后call
			
参数设置:

	小说发表周期:
		ros2 param set /li4 writer_timer_period 
		或者使用rqt设置: 
			Plugins > Configuration > Dynamic Reconfigure > li4
		查看设置结果:
			ros2 param get /li4 writer_timer_period
	二手书单价:
		ros2 param set /wang2 novel_price
		或者使用rqt设置: 
			Plugins > Configuration > Dynamic Reconfigure > wang2
		查看设置结果:
			ros2 param get /wang2 novel_price
			
			
执行launch文件 (该launch文件会同时打开li4_node和wang2_node这两个节点，并设置好参数和命名空间):

	ament_python编译模式下使用python编写的launch文件:
		ros2 launch village_li village.launch.py
	ament_cmake编译模式下使用python编写的launch文件:
		ros2 launch village_wang village.launch.py
	注意: 两种编译模式下使用的village.launch.py文件是一模一样的，只不过前者修改setup.py文件，后者修改CMakeLists.txt文件
	在village.launch.py文件内我们可以修改节点启动时的参数设置和命名空间
	
	
rosbag2:

	打开节点li4_node，使用rosbag2记录话题为/sexy_girl的数据:
		ros2 bag record /sexy_girl                        	(如果不制定bag的名字，就自动以时间命名)
		或者:
		ros2 bag record -o sexy_girl_data /sexy_girl      	(-o可以自定义bag文件的名字)
	ros2 bag存储格式目前仅支持sqllite3, 所以生成了/sexy_girl_data/sexy_girl_data_0.db3文件
	查看bag的相关信息:
		cd sexy_girl_data
		ros2 bag info sexy_girl_data_0.db3
	我们关闭节点，播放话题数据:
		ros2 bag play sexy_girl_data_0.db3			(正常速度播放一次)
		ros2 bag play sexy_girl_data_0.db3 -r 5			(五倍速播放一次)
		ros2 bag play sexy_girl_data_0.db3 -r 5 -l       	(五倍速循环播放)
	我们关闭节点，使用ros2的topic的指令来查看数据
		ros2 topic echo /sexy_girl
	

