#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/*
    订阅方实现
        1. 包含头文件
            ROS中的文本类型 ---> std_msgs/String.h
        2. 初始化ROS节点
        3. 创建节点句柄
        4. 创建订阅者对象
        5. 处理订阅到的布数据
        6. 声明spin()函数
*/
void doMsg(const std_msgs::String::ConstPtr &msg){
    // 参数类型为常量消息的常量指针的引用
    // 通过msg获取并操作订阅到的数据
    ROS_INFO("翠花订阅的数据:%s",msg->data.c_str());

}

int main(int argc, char *argv[])
{
    // 由于日志当中输出中文，所以需要使用setlocale
    setlocale(LC_ALL,"");
    // 2. 初始化ROS节点
    ros::init(argc, argv, "cuiHua");  //节点不能重名
    // 3. 创建节点句柄
    ros::NodeHandle nh;
    // 4. 创建订阅者对象
    ros::Subscriber sub = nh.subscribe("fang",10,doMsg);
    // 5. 处理订阅到的布数据

    // 6. 声明spin()函数
    // spin字面意思就是回头，表示main函数进行到这里会不断回头去执行回调函数
    ros::spin();
    return 0;
}
