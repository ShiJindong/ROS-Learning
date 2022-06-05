#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"                             // 订阅的数据可能很多，需要buffer保存
#include "geometry_msgs/PointStamped.h"                 // 用于创建点的坐标
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"        // 包含buffer.transform()函数

/* 
    需求: 订阅的坐标系相对关系，并且传入一个坐标点，调用tf实现坐标转换
    流程:
        1. 包含头文件
        2. 编码，初始化，NodeHandle(必须的)
        3. 创建订阅关系 ---> 订阅坐标系相对关系
        4. 组织一个坐标点数据
        5. 转换算法，需要调用tf内置实现
        6. 最后输出 
 */

int main(int argc, char *argv[])
{
    // 2. 编码，初始化，NodeHandle(必须的)
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_sub");
    ros::NodeHandle nh;
    // 3. 创建订阅关系 ---> 订阅坐标系相对关系
    // 3-1: 创建一个buffer缓存
    tf2_ros::Buffer buffer;
    // 3-2: 再创建监听对象 (监听对象可以将订阅的数据存入buffer)
    tf2_ros::TransformListener listener(buffer);     // 将buffer传入监听对象
    // 4. 组织一个坐标点数据
    geometry_msgs::PointStamped ps;             // 可在terminal调用 rosmsg info geometry_msgs/PointStamped 查看坐标点的消息格式
    ps.header.frame_id = "laser";               // 该点为laser下的点
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    // 方案一: 添加休眠，保证转换算法执行的时候，坐标变换关系已经被接收了
    // ros::Duration(2).sleep(); 

    // 5. 转换算法，需要调用tf内置实现
    ros::Rate rate(10);
    while(ros::ok())    // 雷达会不间断接受点的消息，这里用while循环进行模拟
    {
        // 核心代码  ---> 将ps转换成相对于base_link的坐标点
        
        /* 
            运行时存在的问题: 抛出异常，base_link不存在
            原因: 订阅数据是一个耗时操作，可能在调用transform转换函数时，坐标系的相对关系还没有订阅到，因此出现异常
            解决:
                方案一: 在调用转换函数transform前，执行休眠
                方案二: 进行异常处理 (建议使用方案二)
        */

        // 方案二: 进行异常处理
        geometry_msgs::PointStamped ps_out;             //  输出结果
        try
        {
            // 需要调用buffer的转换函数，因为坐标变换关系是存储在buffer中的，而不是listener
            // 注意: buffer的函数transform调用时，必须包含头文件 "tf2_geometry_msgs/tf2_geometry_msgs.h"，否则报错
            ps_out = buffer.transform(ps, "base_link");     // 参数1: 被转换的点 参数2: 目标坐标系  返回值: 输出转换后的点
            // 6. 最后输出 
            ROS_INFO("转换后的坐标值: (%.2f, %.2f, %.2f), 参考的坐标系: %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常消息: %s", e.what());
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
