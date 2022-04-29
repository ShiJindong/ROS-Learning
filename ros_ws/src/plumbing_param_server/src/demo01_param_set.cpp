#include "ros/ros.h"

/*
    需要实现参数的新增和修改
    需求:   首先设置机器人的共享参数，类型，半径(0.15m)
            再修改半径(0.2m)

    实现:
        方法一:
        ros::NodeHandle
            setParam
        方法二:
        ros::param
            set()
*/

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"set_param_c");
    ros::NodeHandle nh;
    // 参数新增
    // 方法一
    nh.setParam("type", "xiaoHuang");     // key,  value
    nh.setParam("radius", 0.15);
    // 方法二
    ros::param::set("type_param", "xiaoBai");
    ros::param::set("radius_param", 0.15);

    // 参数修改
    // 方法一
    nh.setParam("radius", 0.2);      // 修改值会覆盖原来的值
    // 方法二
    ros::param::set("radius_param", 0.2);

    return 0;
}
