#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
# from plumbing_server_client.srv import AddInts,AddIntsRequest,AddIntsResponse
from plumbing_server_client.srv import *     # 等价


"""
    服务端: 解析客户端请求，并产生响应。
        1. 导包
        2. 初始化ROS节点
        3. 创建服务端对象
        4. 处理请求，并返回响应 (使用回调函数)
        5. 调用 spin()
"""

# 参数: 封装的请求数据
# 返回值: 响应的数据
def doNum(request):
    #1. 解析请求的数据
    num1 = request.num1
    num2 = request.num2
    #2. 求和
    sum = num1 + num2
    # 3. 将结果封装进响应
    response = AddIntsResponse()
    response.sum = sum

    rospy.loginfo("服务器解析的数据: num1 = %d, num2 = %d, 响应的结果: sum = %d", num1, num2, sum)
    return response

if __name__ == "__main__":
    # 2. 初始化ROS节点
    rospy.init_node("jiSuanJi")
    # 3. 创建服务端对象
    server = rospy.Service("addInts",AddInts,doNum)
    rospy.loginfo("服务器已经启动了")
    # 4. 处理请求，并返回响应 (使用回调函数)
    # 5. 调用 spin()
    rospy.spin()