#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from plumbing_pub_sub.msg import Person
"""
    发布方: 发布人的消息
        1. 导包
        2. 初始化ROS节点
        3. 创建发布者对象
        4. 组织发布逻辑并发布数据
"""

if __name__ == "__main__":
    # 2. 初始化ROS节点
    rospy.init_node("daMa")
    # 3. 创建发布者对象
    pub = rospy.Publisher("liaoTian",Person,queue_size=10)
    # 4. 组织发布逻辑并发布数据
    # 4.1 创建Person数据
    p = Person()
    p.name = "Ultraman"
    p.age = 0
    p.height = 100
    # 4.2 创建Rate对象
    rate = rospy.Rate(1)
    # 4.3 循环发布
    while not rospy.is_shutdown():
        p.age += 1
        pub.publish(p)
        rospy.loginfo("发布的消息:%s,%d,%.2f",p.name, p.age, p.height)
        rate.sleep()
