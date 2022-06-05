#! /usr/bin/env python
# -*- coding: UTF-8 -*-

from datetime import date
import rospy
import rosbag
from std_msgs.msg import String    # 发布的消息的类型

""" 
    需求: 使用rosbag读取磁盘文件中的数据(话题 + 消息)
    流程:
        1. 导包
        2. 初始化
        3. 创建rosbag 对象, 并打开文件流 (以读的方式打开)
        4. 读取数据
        5. 关闭文件流
 """

if __name__ == "__main__":
    # 2. 初始化
    rospy.init_node("read_bag_p")
    # 3. 创建rosbag 对象, 并打开文件流 (以读的方式打开)
    bag = rosbag.Bag("hello_p.bag",'r')
    # 4. 读取数据
    msgs = bag.read_messages("/liaoTian")
    #  函数read_messages()的返回值
    #  @return: generator of BagMessage(topic, message, timestamp) namedtuples for each message in the bag file
    for topic,msg,time in msgs:
        rospy.loginfo("解析的内容: 话题: %s, 消息值: %s, 时间戳: %s",topic,msg.data,time)  # 注意time类型为string

    # 5. 关闭文件流
    bag.close()
