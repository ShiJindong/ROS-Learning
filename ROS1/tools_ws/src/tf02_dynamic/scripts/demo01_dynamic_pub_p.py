#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
# 调用 tf.transformations 和 tf_conversions 其一即可
import tf.transformations   # 用于调用 tf.transformations.quaternion_from_euler(0,0,0)
# 或
import tf_conversions       # 用于调用 tf_conversions.transformations.quaternion_from_euler(0,0,0)

""" 
    发布方: 需要订阅乌龟的位姿信息，转换为相对于世界坐标系(即窗体)的坐标关系，并发布
    准备:
        话题名称: /turtle1/pose
        消息类型: /turtlesim/Pose
    流程:
        1. 包含头文件
        2. 节点初始化， NodeHandle
        3. 创建订阅对象，订阅 /turtle1/pose
        4. 回调函数处理订阅的消息: 将位姿信息转换成世界坐标并发布
        5. spin()

    查看: 在terminal输入  rostopic list     
                        rostopic echo /tf
         打开 rviz, 选择 world, 添加 tf
         控制小龟运动，查看坐标变换关系的变化
"""

def doPose(pose):
    # a. 创建发布对象
    pub = tf2_ros.TransformBroadcaster()
    # b. 组织被发布的数据
    tfs = TransformStamped()
    # header
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = "world"
    # child frame
    tfs.child_frame_id = "turtle1"
    # 相对关系
    # 设置偏移量
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0.0
    # 设置旋转量
    """ 
        乌龟位姿信息中没有四元数, 但是有个偏航角度,
        又已知乌龟是2D的, 所以乌龟的欧拉角为(roll, pitch, yaw) = (0,0,theta)
    """
    # 4-1: 先从欧拉角转换成四元数
    # qtn = tf_conversions.transformations.quaternion_from_euler(0,0,pose.theta)
    qtn = tf.transformations.quaternion_from_euler(0,0,pose.theta)
    # 4-2: 再设置四元数
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    # 5. 发布数据
    pub.sendTransform(tfs)

if __name__ == "__main__":
    # 2. 初始化节点
    rospy.init_node("dynamic_pub_p")
    # 3. 创建发布对象
    sub = rospy.Subscriber("/turtle1/pose",Pose,doPose,queue_size=100)
    # 4. 回调函数处理订阅的消息: 将位姿信息转换成世界坐标并发布
    rospy.loginfo("动态坐标变换关系正在发布!")
    # 5. spin()
    rospy.spin()