#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

"""
    需求: 向服务器发送请求，生成一只新的乌龟
        话题: /spawn
        消息类型: turtlesim/Spawn

    实现:
        1. 导包
        2. 初始化ROS节点
        3. 创建客户端对象
        4. 组织数据，发送请求，处理响应
        5. 不需要回调函数
"""


if __name__ == "__main__":
    #  2. 初始化ROS节点
    rospy.init_node("service_call_p")
    #  3. 创建客户端对象
    client = rospy.ServiceProxy("/spawn",Spawn)
    #  4. 组织数据，发送请求，处理响应
    request = SpawnRequest()
    request.x = 4.5
    request.y = 2.0
    request.theta = -3
    request.name = "turtle3"

    # 判断服务器状态，再发送
    client.wait_for_service()
    # 使用try进行异常处理
    try:
        responce = client.call(request)   # 与cpp中不同，这里函数call会返回responce
        # 处理响应结果
        rospy.loginfo("新生成的乌龟叫: %s", responce.name)
    except Exception as e:
        rospy.logerr("请求处理异常!")