#include "ros/ros.h"

/*
    演示参数删除
    实现:
        方法一:
        ros::NodeHandle
            deleteParam()

        方法二:
        ros::param
            del()
*/

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"param_del_c");
    ros::NodeHandle nh;

    // 方法一
    bool flag1 = nh.deleteParam("radius");
    if(flag1)
        ROS_INFO("delete radius success!");
    else    
        ROS_INFO("delete radius not success!");         // 只能删除一次，第二次删除会报失败!

    // 方法二
    bool flag2 = ros::param::del("radius_param");
    if(flag2)
        ROS_INFO("delete radius_param success!");   
    else    
        ROS_INFO("delete radius_param not success!");   // 只能删除一次，第二次删除会报失败!

    return 0;
}
