#include "ros/ros.h"
#include "std_msgs/String.h"


/*
    需求: 演示不同类型的话题名称设置
            设置话题与命名空间
*/

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"topic_name");

    // 设置不同类型的话题
    // terminal 内执行 rosrun rename02_topic topic_name __ns:=xxx ，根据 rosnode list, rostopic list 

    // 1 全局  ---  话题名需要以斜杠开头，这种情况下与节点命名空间和节点名称没有关系，它也可以设置自己的命名空间
    // 设置全局话题
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter", 1000);          // 比如若节点命名空间叫xxx，则话题名为 /chatter
    // 设置全局话题自己的命名空间
    // ros::Publisher pub = nh.advertise<std_msgs::String>("/yyy/chatter", 1000);   // 比如若节点命名空间叫xxx，则话题名为 /yyy/chatter

    // 2 相对  --- 话题名不带斜杠，最终的话题与节点命名空间和节点名称有关
    // ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);        // 比如若节点命名空间叫xxx，则话题名为 /xxx/chatter
    // 设置相对话题自己的命名空间
    // os::Publisher pub = nh.advertise<std_msgs::String>("yyy/chatter", 1000);     // 比如若节点命名空间叫xxx，则话题名为 /xxx/yyy/chatter

    // 3 私有  --- 需要创建特定 NodeHandle, 设置参数为"~",即可设为该节点私有的话题
    // ros::NodeHandle nh("~");
    // ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);        // 比如若节点命名空间叫xxx，则话题名为 /xxx/topic_name/chatter,属于topic_name这个节点私有的话题名称
    // ros::Publisher pub = nh.advertise<std_msgs::String>("yyy/chatter", 1000);

    // 全局话题优先级 高于 私有话题
    // ros::NodeHandle nh("~");
    // ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter", 1000);   // 比如若节点命名空间叫xxx，则话题名/xxx/chatter, 话题名前面带斜杠后，不再是私有话题，而是全局话题

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
