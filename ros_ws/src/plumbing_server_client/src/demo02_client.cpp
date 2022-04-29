#include "ros/ros.h"
#include "plumbing_server_client/AddInts.h"

/*
    客户端实现: 客户端需要提交两个整数，并处理响应的结果
        1. 包含头文件
        2. 初始化ROS节点
        3. 创建节点句柄
        4. 创建一个客户端对象
        5. 提交请求，并处理响应
        6. 由于没有回调函数，所以spin()可有可无

    实现参数的动态提交
        1. 格式: 比如 rosrun plumbing_server_client demo02_client 12 34
        2. 节点执行时需要获取命令中的参数，并组织进request

    问题: 
        如果先启动客户端，不启动服务器端，那么会请求异常
    而我们的需求是:
        如果先启动客户端，不启动服务器端，不要直接抛出异常，而是挂起，等服务器启动后，再正常请求服务
        这是非常常见的一个需求，因为有时候我们需要启动的节点有很多，不能保证服务端一定是开启的
    解决:
        在ROS中内置了相关函数，可以让客户端启动后挂起，去等待服务器启动, 可以使用函数1或函数2
        函数1: client.waitForExistence();
        函数2: ros::service::waitForService("addInts");

*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 优化实现，获取命令中的参数
    if(argc != 3){
        ROS_INFO("提交的参数个数不对！");
        return 1;
    }

    // 2. 初始化ROS节点
    ros::init(argc,argv,"daBao");
    // 3. 创建节点句柄
    ros::NodeHandle nh;
    // 4. 创建一个客户端对象
    ros::ServiceClient client = nh.serviceClient<plumbing_server_client::AddInts>("addInts");
    // 5. 提交请求，并处理响应
    // 5.1 组织请求
    plumbing_server_client::AddInts ai;
    ai.request.num1 = atoi(argv[1]);    // argv[0]包含程序名 atoi: char to integer
    ai.request.num2 = atoi(argv[2]);

    // 5.2 处理响应

    // 调用判断服务器状态的函数
    // 函数1: 实现客户端在服务器未启动情况下的挂起
    // client.waitForExistence();
    // 函数2: 也能实现和函数1一样的功能
    ros::service::waitForService("addInts");

    bool flag = client.call(ai);     // 客户端client通过call (topic为addInts)访问服务器，同时需要提交请求ai,内含请求计算求和的数字

    if(flag)
    {
        ROS_INFO("响应成功！");
        // 获取结果
        ROS_INFO("响应结果 = %d", ai.response.sum);
    }
    else{
        ROS_INFO("响应失败！");
    }
    // 6. 由于没有回调函数，所以spin()可有可无
    return 0;
}
