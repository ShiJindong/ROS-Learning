#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include <sstream>

/* 
    需求: 使用rosbag向磁盘文件写入数据(话题 + 消息)
    流程:
        1. 导包
        2. 初始化
        3. 创建rosbag 对象
        4. 打开文件流 (以写的方式打开)
        5. 写入数据
        6. 关闭文件流
 */

int main(int argc, char *argv[])
{
    // 2. 初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag_write");
    ros::NodeHandle nh;
    // 3. 创建rosbag 对象
    rosbag::Bag bag;
    // 4. 打开文件流 (以写的方式打开)
    bag.open("hello.bag",rosbag::BagMode::Write);
    // 5. 写入数据
    std_msgs::String msg;

    for(int i = 0; i < 1000; ++i)
    {
        // 实现字符串的拼接, 使用 stringstream
        std::stringstream ss;
        ss << "hello ---> " << i;
        msg.data = ss.str();
        bag.write("/chatter",ros::Time::now(),msg);
         /* 
            函数 write:
                参数1: 话题名
                参数2: 时间戳
                参数3: 消息数据
        */
    }

    // 6. 关闭文件流
    bag.close();
    return 0;
}
