#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import tf2_ros
# 调用 tf.transformations 和 tf_conversions 其一即可
import tf.transformations   # 用于调用 tf.transformations.quaternion_from_euler(0,0,0)
# 或
import tf_conversions       # 用于调用 tf_conversions.transformations.quaternion_from_euler(0,0,0)
from geometry_msgs.msg import TransformStamped


""" 
     发布方: 发布两个坐标系的相对关系(车辆底盘base_link 和 雷达laser)
     流程:
        1. 导包
        2. 初始化节点
        3. 创建发布对象
        4. 组织被发布的数据
        5. 发布数据
        6. spin()

    查看: 在terminal输入  rostopic list     
                        rostopic echo /tf_static
         打开 rviz, 选择 base_link, 添加 tf
"""

if __name__ == "__main__":
    # 2. 初始化节点
    rospy.init_node("static_pub_p")
    # 3. 创建发布对象
    pub = tf2_ros.StaticTransformBroadcaster()
    # 4. 组织被发布的数据
    tfs = TransformStamped()
    # header
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = "base_link"
    # child frame
    tfs.child_frame_id = "laser"
    # 相对关系
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5
    # 4-1: 先从欧拉角转换成四元数
    # qtn = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    # 4-2: 再设置四元数
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    # 5. 发布数据
    pub.sendTransform(tfs)
    rospy.loginfo("静态坐标变换关系已经发布!")
    # 6. spin()
    rospy.spin()