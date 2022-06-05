#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String    # 发布的消息的类型
import tools
import os
import sys

# Noetic rosrun时 (melodic没有此种问题)，ros可能找不到引用的外部包的路径，因为它的参考路径是workspace，解决方法有两种

# 方法一:
# 设置临时环境变量
# sys.path.insert(0, "/home/jindong/ROS/ros_ws/src/plumbing_pub_sub/scripts")

# 方法二:
# 路径写死，影响了代码的可移植性
# 优化，可以动态获取路径
path = os.path.abspath(".")
rospy.loginfo("执行时参考的路径: %s",path)
sys.path.insert(0,path + "/src/plumbing_pub_sub/scripts")


"""
    使用python实现消息发布
        1. 导包
        2. 初始化ros节点
        3. 创建发布者对象
        4. 编写发布逻辑并发布数据
"""

if __name__ == "__main__":

    # 2. 初始化ros节点
    rospy.init_node("sanDai")    #传入节点名称
    # 3. 创建发布者对象
    pub = rospy.Publisher("fang",String,queue_size=10)

    rospy.loginfo("num = %d", tools.num)

    # 4. 编写发布逻辑并发布数据
    # 创建数据
    msg = String()    #空的String
    # 指定发布频率
    rate = rospy.Rate(1)
    # 设置计数器
    count = 0
    # 在节点开始运行后，延迟3秒发布数据，防止订阅者在发布者注册期间丢失数据
    rospy.sleep(3)
    # 使用循环发布数据
    while not rospy.is_shutdown():
        count += 1
        msg.data = "hello" + str(count)
        # 发布数据
        pub.publish(msg)
        rospy.loginfo("发布的数据是:%s",msg.data)
        rate.sleep()