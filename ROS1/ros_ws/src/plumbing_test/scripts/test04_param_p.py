#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy

""""
    需求: 修改参数服务器中 turtlesim 背景颜色相关的参数

    实现:
        1. 初始化ROS节点
        2. 不一定需要创建节点句柄(和后续API有关)  方法一 nh  方法二 ros::param
        3. 修改参数
"""

if __name__ == "__main__":
    rospy.init_node("change_bgColor_p")
    rospy.set_param("/turtlesim/background_r",0)
    rospy.set_param("/turtlesim/background_g",255)
    rospy.set_param("/turtlesim/background_b",0)