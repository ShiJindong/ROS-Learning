#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
# from plumbing_server_client.srv import AddInts,AddIntsRequest,AddIntsResponse
from plumbing_server_client.srv import *     # 等价
import sys
import logging     # 使用了rospy.logerr
logging.basicConfig()

"""
    客户端: 组织并提交请求，并处理服务端的响应。
        1. 导包
        2. 初始化ROS节点
        3. 创建客户端对象
        4. 组织请求的数据，并发送请求 
        5. 处理响应

    优化实现:
        可以在执行节点时，动态传入参数

    需求:
        判断服务器的状态，如果服务器未启动，就让客户端挂起，等待服务器的启动，而不是直接抛出异常
        实现: ROS中已经内置了相关函数,总共两种方法
              方法一: client.wait_for_service()
              方法二: rospy.wait_for_service("addInts")
"""

if __name__ == "__main__":
    # 判断传入的参数是否为3个
    if len(sys.argv) != 3:
        rospy.logerr("传入的参数个数不对!")
        sys.exit(1)
    #  2. 初始化ROS节点
    rospy.init_node("xueSheng")
    #  3. 创建客户端对象
    client = rospy.ServiceProxy("addInts",AddInts)   # 话题名称，服务数据类型
    rospy.loginfo("客户端对象已经创建了!")
    #  4. 组织请求的数据，并发送请求 
    # 解析传入的参数
    num1 = int(sys.argv[1])   # argv是数组，其中包含程序名和两个数字，应该将它们数字转化为int类型
    num2 = int(sys.argv[2])

    # 等待服务端启动
    # 方法一:
    # client.wait_for_service()
    # 方法二:
    rospy.wait_for_service("addInts")

    responce = client.call(num1,num2)
    #  5. 处理响应
    rospy.loginfo("响应的数据:%d",responce.sum)