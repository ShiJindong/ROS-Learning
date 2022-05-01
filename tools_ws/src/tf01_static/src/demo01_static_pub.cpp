#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"   // 包含静态坐标关系发布方 tf2_ros::StaticTransformBroadcaster
#include "geometry_msgs/TransformStamped.h"         // 包含发布的坐标变换关系 geometry_msgs::TransformStamped
#include "tf2/LinearMath/Quaternion.h"              // 包含四元数 tf2::Quaternion
/* 
    需求: 发布两个坐标系的相对关系
    流程:
        1. 包含头文件
        2. 设置编码 节点初始化 NodeHandle(非必须)
        3. 创建发布对象
        4. 组织被发布的消息
        5. 发布数据 (静态坐标发一次就可以了)
        6. spin();

    查看: 在terminal输入  rostopic list     
                        rostopic echo /static_pub
         打开 rviz, 选择 base_link, 添加 tf
 */

int main(int argc, char *argv[])
{
    // 2. 设置编码 节点初始化 NodeHandle(非必须)
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub");
    // 3. 创建发布对象
    tf2_ros::StaticTransformBroadcaster pub;
    // 4. 组织被发布的消息
    geometry_msgs::TransformStamped tfs;        // 坐标变换关系
    tfs.header.seq = 100;                       // 设置序列化号，可以不设置，也可以随便设置一个
    tfs.header.stamp = ros::Time::now();        // 设置时间戳，即发布的时间
    tfs.header.frame_id = "base_link";          // 相对坐标系关系中，被参考的那一个
    tfs.child_frame_id = "laser";               // laser相对于base_link
    tfs.transform.translation.x = 0.2;          // 可在terminal调用 rosmsg info geometry_msgs/TransformStamped 查看坐标变换关系的消息格式
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    // 需要根据欧拉角转换
    tf2::Quaternion qtn;                        // 创建四元数对象
    // 向该对象设置欧拉角，这个对象可以将欧拉角转换为四元数
    // (函数setRPY的参数为欧拉角，其单位为弧度)
    qtn.setRPY(0,0,0);      // 如果雷达安装倒了就设成3.14159
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    // 5. 发布数据 (静态坐标发一次就可以了)
    pub.sendTransform(tfs);
    ROS_INFO("静态坐标变换关系已发布!");
    // 6. spin();
    ros::spin();

    return 0;
}
