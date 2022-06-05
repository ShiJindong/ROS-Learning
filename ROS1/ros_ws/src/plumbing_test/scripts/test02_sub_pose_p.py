#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from turtlesim.msg import Pose

"""
    使用python处理订阅到的位置消息
  

    实现:
        1. 导包
        2. 初始化ros节点
        3. 创建订阅者对象
        4. 编写订阅逻辑并处理数据
        5. spin()
"""

def doPose(pose):
    rospy.loginfo("乌龟的位置信息: 坐标 (%.2f,%.2f), 朝向: %.2f, 线速度: %.2f, 角速度: %.2f",
                    pose.x, pose.y, pose.theta,pose.linear_velocity, pose.angular_velocity)

if __name__ == "__main__":
    rospy.init_node("sub_pose_p")
    sub = rospy.Subscriber("/turtle1/pose",Pose,doPose,queue_size=100)
    rospy.spin()