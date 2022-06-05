#include "ros/ros.h"
#include "std_msgs/String.h"


/*
    需求: 演示不同类型的参数设置
*/

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"param_name");
    /* 
        使用 ros::param 来设置参数
     */

    // 1 全局参数  ---  参数名需要以斜杠开头，这种情况下与节点命名空间和节点名称没有关系，它也可以设置自己的命名空间
    ros::NodeHandle nh;
    ros::param::set("/radiusA",100);            // 假如设置该节点的命名空间为xxx，则该全局参数为  /radiusA

    // 2 相对参数
    ros::param::set("radiusA",100);             // 假如设置该节点的命名空间为xxx，则该相对参数  /xxx/radiusA

    // 3 私有参数
    ros::param::set("~radiusA",100);            // 假如设置该节点的命名空间为xxx，则该私有参数  /xxx/param_nbame/radiusA


    /* 
        使用 NodeHandle
     */
    // 1 全局参数
    nh.setParam("/radius_nh_A",1000);           // 假如设置该节点的命名空间为xxx，则该全局参数为  /radius_nh_A

    // 2 相对参数
    nh.setParam("radius_nh_A", 1000);           // 假如设置该节点的命名空间为xxx，则该相对参数  /xxx/radius_nh_A

    // 3 私有参数
    ros::NodeHandle nh_private("~");
    nh_private.setParam("radius_nh_A", 1000);   // 假如设置该节点的命名空间为xxx，则该私有参数  /xxx/param_nbame/radius_nh_A

    return 0;
}