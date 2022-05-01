#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"  
#include "geometry_msgs/Twist.h"

/* 
    需求:  
        1. 计算turtle1相对于turtle2的坐标关系  
        2. 计算turtle2的线速度和角速度并发布

 */

int main(int argc, char *argv[])
{
    // 2. 编码，节点初始化，NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"control_turtle2");
    ros::NodeHandle nh;
    // 3. 创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);

    // 4. 编写解析逻辑: 设置循环
    // A. 创建发布速度的对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",100);
    ros::Rate rate(10);
    while(ros::ok())
    {
        try
        {
            // 1. 计算son1相对于son2的相对坐标关系
            geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));  
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

            ROS_INFO("turtle1 相对于 turtle2 的相对变换关系: 父级: %s, 子级: %s, 偏移量: (%.2f, %.2f, %.2f)",
                            son1Toson2.header.frame_id.c_str(),     // turtle2
                            son1Toson2.child_frame_id.c_str(),      // turtle1
                            son1Toson2.transform.translation.x,
                            son1Toson2.transform.translation.y,
                            son1Toson2.transform.translation.z);   

            // B. 根据相对变换关系，计算速度
            /* 
            组织速度，只需要设置线速度的x 和 角速度的z
                 线速度的x = 系数 * sqrt(x*x + y*y)
                 角速度的z = 系数 * atan2(y, x)
            */
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5 * std::sqrt(std::pow(son1Toson2.transform.translation.x,2) + 
                                   std::pow(son1Toson2.transform.translation.y,2));
            twist.angular.z = 4 * std::atan2(son1Toson2.transform.translation.y,
                                             son1Toson2.transform.translation.x);
            // C. 发布速度指令
            pub.publish(twist);
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
