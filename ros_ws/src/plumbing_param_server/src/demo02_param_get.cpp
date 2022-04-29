#include "ros/ros.h"

/*
    演示参数查询
    实现:
        方法一:
        ros::NodeHandle
            param(键,默认值) 
            存在，返回对应结果，否则返回默认值

            getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

            hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

            searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量
        方法二:
        ros::param
             -----> 与 NodeHandle 类似

            
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"get_param_c");
    ros::NodeHandle nh;
    // 方法一:
    // 1. param(键,默认值) 
    double radius = nh.param("radius", 0.5);
    ROS_INFO("radius = %.2f", radius);

    // 2. getParam
    double radius2 = 0.0;
    bool result2 = nh.getParam("radius", radius2);
    if(result2)
    {
        ROS_INFO("radius2 = %.2f", radius2);
    }
    else{
        ROS_INFO("radius2 does not exist!");
    }

    // 3. getParamCached 与getParam结果一致，只不过性能有提升
    double radius3 = 0.0;
    bool result3 = nh.getParamCached("radius", radius3);
    if(result3)
    {
        ROS_INFO("radius3 = %.2f", radius3);
    }
    else{
        ROS_INFO("radius3 does not exist!");
    }

    // 4. getParamNames 获取键
    std::vector<std::string> names;
    nh.getParamNames(names);
    for(auto &name: names)
        ROS_INFO("name: %s", name.c_str());

    // 5. hasParam
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("radiusxxx");
    ROS_INFO("Does radius exist? %d", flag1);
    ROS_INFO("Does radiusxxx exist? %d", flag2);

    // 6. searchParam
    std::string key1, key2;
    nh.searchParam("radius", key1);
    ROS_INFO("Search result for radius: %s", key1.c_str());
    nh.searchParam("radiusxxx", key2);
    ROS_INFO("Search result for radiusxxx: %s", key2.c_str());


    // 方法二
    // 使用ros::param，只要nh有的函数，其中都有类似的
    double radius_param = ros::param::param("radius", 0.6);
    ROS_INFO("radius_param = %.2f", radius_param);

    return 0;
}
