#include "ros/ros.h"

/*
    需求: 修改参数服务器中 turtlesim 背景颜色相关的参数

    实现:
        1. 初始化ROS节点
        2. 不一定需要创建节点句柄(和后续API有关)  方法一 nh  方法二 ros::param
        3. 修改参数
*/

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"change_bgColor");

    // 方法一 使用句柄
    // ros::NodeHandle nh;
    // nh.setParam("background_r",255);
    // nh.setParam("background_g",255);
    // nh.setParam("background_b",255);

    // 使用句柄，只不过与前者稍有不同的是使用带有命名空间的句柄
    ros::NodeHandle nh("turtlesim");
    nh.setParam("/turtlesim/background_r",0);
    nh.setParam("/turtlesim/background_g",50);
    nh.setParam("/turtlesim/background_b",100);

    // 方法二 使用 ros::param
    // ros::param::set("/turtlesim/background_r", 0);
    // ros::param::set("/turtlesim/background_g", 0);
    // ros::param::set("/turtlesim/background_b", 0);

    return 0;
}
