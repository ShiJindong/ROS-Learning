#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

""" 
     订阅方: 需求: 订阅的坐标系相对关系, 并且传入一个坐标点,调用tf实现坐标转换
     流程:
        1. 导包
        2. 初始化节点
        3. 创建订阅对象
        4. 组织被转换的坐标点
        5. 转换逻辑实现,调用tf封装的算法
        6. 输出转换的结果
"""

if __name__ == "__main__":
    # 2. 初始化节点
    rospy.init_node("dynamic_sub_p")
    # 3. 创建订阅对象
    #3-1: 创建缓存对象
    buffer = tf2_ros.Buffer()
    #3-2: 创建订阅对象，并将缓存传入
    sub = tf2_ros.TransformListener(buffer)
    # 4. 组织被转换的坐标点
    ps = tf2_geometry_msgs.PointStamped()         # 可在terminal调用 rosmsg info geometry_msgs/PointStamped 查看坐标点的消息格式
    
    ps.header.stamp = rospy.Time()
    """ 
        ps.header.stamp = rospy.Time.now() 
        这样设置点的时间戳,系统会报错: Requested time..., but the earlliest date is..., 
                               when looking up transform form frame turtle1 to frame world!
        报错原因:
            buffer里面存储的动态坐标转换关系是包含有一个时间戳的,并且该时间戳是实时变动的
            而被转换的坐标点ps也有一个时间戳,而坐标转换的时候,ros会拿这个点的时间戳和变换关系的时间戳进行比对,
            比对的时候会判断,两个时间戳的时间是否是接近的,如果两者差的太多,ros认为两者不相匹配,就会报错
        解决: 
            ps.header.stamp = rospy.Time()
            而倘若将被转换的点的时间戳设为0的话,ros就认为时间戳的判断无所谓了,不会去报错
    """
    
    ps.header.frame_id = "turtle1"
    ps.point.x = 2.0
    ps.point.y = 3.0
    ps.point.z = 5.0

    # 5. 转换逻辑实现,调用tf封装的算法
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():     # 雷达会不间断接受点的消息，这里用while循环进行模拟
         
        """  运行时存在的问题: 抛出异常, base_link不存在
            原因: 订阅数据是一个耗时操作, 可能在调用transform转换函数时, 坐标系的相对关系还没有订阅到, 因此出现异常
            解决:
                方案一: 在调用转换函数transform前, 执行休眠
                方案二: 进行异常处理 (建议使用方案二) 
        """
        # 方案二: 进行异常处理
        try:
            # 转换实现
            ps_out = buffer.transform(ps,"world")   # 需要两个参数(被转换的坐标点和目标坐标系),  返回值为转换后的坐标点
            # 6. 输出转换的结果
            rospy.loginfo("转换后的坐标: (%.2f, %.2f, %.2f), 参考的坐标系: %s",
                            ps_out.point.x,
                            ps_out.point.y,
                            ps_out.point.z,
                            ps_out.header.frame_id)
        except Exception as e:
            rospy.logwarn("错误提示:%s", e)
       
        rate.sleep()