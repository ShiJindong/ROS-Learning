#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"  
/* 
    订阅方实现:  1. 计算son1和son2的相对关系  2. 计算son1中的某个坐标点在son2中的坐标值
    实现流程:
        1. 包含头文件
        2. 编码，节点初始化，NodeHandle
        3. 创建订阅对象
        4. 编写解析逻辑: 设置循环
        5. spinOnce()

    执行:
        roslaunch tf03_tfs tfs_c.launch    ---> 发布 son1 相对于 world， 以及 son2 相对于 world 的坐标关系
        rosrun tf03_tfs demo01_tfs         ---> 1. 计算son1和son2的相对关系  2. 计算son1中的某个坐标点在son2中的坐标值

 */

int main(int argc, char *argv[])
{
    // 2. 编码，节点初始化，NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub");
    ros::NodeHandle nh;
    // 3. 创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    // 4. 编写解析逻辑: 设置循环
    // 创建坐标点
    geometry_msgs::PointStamped psAtSon1;
    psAtSon1.header.stamp = ros::Time::now();
    psAtSon1.header.frame_id = "son1";
    psAtSon1.point.x = 1.0;
    psAtSon1.point.y = 2.0;
    psAtSon1.point.z = 3.0;

    ros::Rate rate(10);
    while(ros::ok())
    {
        try
        {
            // 1. 计算son1相对于son2的相对坐标关系
            geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("son2","son1",ros::Time(0));  
            /* 
                需求: 计算 A 相对于 B 的相对坐标关系  
                    --->  使用 buffer.lookupTransform("B","A",ros::Time(0))
                参数1: 目标坐标系
                参数2: 源坐标系
                参数3: ros::Time(0) 取间隔最短的两个坐标关系帧间来计算坐标相对关系
                            因为订阅到的son1和son2的坐标关系并不一定是同时刻发布的，两者之间可能存在延迟
                            传入参数 ros::Time(0)，那么函数lookupTransform会查看发表时刻挨的最近的两个相对关系
                返回值: geometry_msgs::TransformStamped 源坐标系 相对于 目标坐标系的相对变换关系
             */ 

            ROS_INFO("son1 相对于 son2 的相对变换关系: 父级: %s, 子级: %s, 偏移量: (%.2f, %.2f, %.2f)",
                            son1Toson2.header.frame_id.c_str(),     // son2
                            son1Toson2.child_frame_id.c_str(),      // son1
                            son1Toson2.transform.translation.x,
                            son1Toson2.transform.translation.y,
                            son1Toson2.transform.translation.z);   
            // 2. 计算son1中的某个坐标点在son2中的坐标值
            geometry_msgs::PointStamped psAtSon2 = buffer.transform(psAtSon1, "son2");
            ROS_INFO("坐标点在son2中的值: (%.2f, %.2f, %.2f)",
                            psAtSon2.point.x,
                            psAtSon2.point.y,
                            psAtSon2.point.z);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("错误提示: %s", e.what());
        }
        

        rate.sleep();
        ros::spinOnce();
    }
    // 5. spinOnce()
    return 0;
}
