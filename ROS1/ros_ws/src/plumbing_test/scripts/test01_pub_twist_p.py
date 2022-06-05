#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist

"""
    使用python实现消息发布
    话题名: /turtle1/cmd_vel
    消息: geometry_msgs/Twist

    实现:
        1. 导包
        2. 初始化ros节点
        3. 创建发布者对象
        4. 编写发布逻辑并发布数据
"""

if __name__ == "__main__":
    rospy.init_node("my_control_p")
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist, queue_size=10)
    # 设置发布频率
    rate = rospy.Rate(10)
    #创建消息
    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.5
    #循环发布
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()