#include "ros/ros.h"
#include "plumbing_head/hello.h"

namespace hello_ns {

void MyHello::run(){
    ROS_INFO("可执行文件中的run 函数执行.....");
}

}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv, "hello_head");
    hello_ns::MyHello myHello;
    myHello.run();

    return 0;
}
