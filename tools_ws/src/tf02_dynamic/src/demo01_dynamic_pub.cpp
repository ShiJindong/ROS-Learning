#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"     // tf2_ros/transform_broadcaster.h用来发布动态坐标变换关系，有别于 tf2_ros/static_transform_broadcaster.h，     
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"         
/* 
    发布方: 需要订阅乌龟的位姿信息，转换为相对于世界坐标系(即窗体)的坐标关系，并发布
    准备:
        话题名称: /turtle1/pose
        消息类型: /turtlesim/Pose
    流程:
        1. 包含头文件
        2. 设置编码，节点初始化， NodeHandle
        3. 创建订阅对象，订阅 /turtle1/pose
        4. 回调函数处理订阅的消息: 将位姿信息转换成世界坐标并发布
        5. spin()

    查看: 在terminal输入  rostopic list     
                        rostopic echo /tf
         打开 rviz, 选择 world, 添加 tf
         控制小龟运动，查看坐标变换关系的变化

 */

void doPose(const turtlesim::Pose::ConstPtr &pose)
{
    // 获取位姿信息, 转换成世界坐标，并发布
    // a. 创建发布对象
    static tf2_ros::TransformBroadcaster pub;    // 注意: 设置为静态对象，这样每次执行回调函数，使用的都是同一个对象，而不是生成一个新的对象
    // b. 组织被发布的数据
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "turtle1";
    // 坐标系偏移量设置
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0;
    // 坐标系旋转量设置
    /* 
        乌龟位姿信息中没有四元数，但是有个偏航角度，又已知乌龟是2D的，所以乌龟的欧拉角为(roll, pitch, yaw) = (0,0,theta)
     */
    tf2::Quaternion qtn;
    qtn.setEuler(0,0,pose->theta);

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    
    // c. 发布
    pub.sendTransform(tfs);

}

int main(int argc, char *argv[])
{
    // 2. 设置编码，节点初始化， NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "dynamic_pub");
    ros::NodeHandle nh;
    // 3. 创建订阅对象，订阅 /turtle1/pose
    ros::Subscriber sub = nh.subscribe("/turtle1/pose",100,doPose);
    // 4. 回调函数处理订阅的消息: 将位姿信息转换成世界坐标并发布
    ROS_INFO("动态坐标变换关系正在发布!");
    // 5. spin()
    ros::spin();
    


    return 0;
}
