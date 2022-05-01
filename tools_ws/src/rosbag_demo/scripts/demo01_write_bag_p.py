#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import rosbag
from std_msgs.msg import String    # 发布的消息的类型

""" 
    需求: 使用rosbag向磁盘文件写入数据(话题 + 消息)
    流程:
        1. 导包
        2. 初始化
        3. 创建rosbag 对象, 并打开文件流 (以写的方式打开)
        4. 写入数据
        5. 关闭文件流
 """

if __name__ == "__main__":
    # 2. 初始化
    rospy.init_node("write_bag_p")
    # 3. 创建rosbag 对象, 并打开文件流 (以写的方式打开)
    bag = rosbag.Bag("hello_p.bag",'w')
    # 4. 写入数据
    msg = String()
    for i in range(1000):
        msg.data = "hello --->" + str(i)
        bag.write("/liaoTian",msg,rospy.Time.now())
    # 5. 关闭文件流
    bag.close()
