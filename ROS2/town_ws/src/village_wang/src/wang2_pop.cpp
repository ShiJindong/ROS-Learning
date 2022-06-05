#include "rclcpp/rclcpp.hpp"

/*  
    创建工作空间
        mkdir -p town_ws/src
        cd town_ws/src
    创建cpp版本的功能包
        ros2 pkg create village_wang --build-type ament_cmake --dependencies rclcpp  
        或
        ros2 pkg create village_wang --dependencies rclcpp   
            kg create 是创建包的意思
            --build-type 用来指定该包的编译类型，一共有三个可选项ament_python、ament_cmake、cmake
            --dependencies 指的是这个功能包的依赖，这里给了一个ros2的cpp客户端接口rclcpp
            如果build-type什么都不写，ros2会默认类型为ament_cmake.
    创建节点文件
        src目录下创建cpp文件

    注意: 智能提示需要先在vscode中安装C/C++这个插件
         另外，头文件"rclcpp/rclcpp.hpp"包含出错，这不是编译报错，只是编辑器报错
         可能是没有给定头文件的路径
         可以鼠标靠近include,点击黄色灯泡，选择include path,
         然后会自动打开C/C++ Configurations, 找到include path一栏，填入/opt/ros/foxy/**
         然后就会生成c_cpp_properties.json文件，用来配置头文件路径
         就不会报错了
*/ 


// 使用面向过程(POP)的思想来构建节点

/* 
    ros2运行该节点的 入口函数
        编写ROS2节点的一般步骤
        1. 导入库文件
        2. 初始化客户端库
        3. 新建节点对象
        4. spin循环节点
        5. 关闭客户端库
        6. 在CMakeLists.txt中加入配置，使得其可以找到该节点
                在CmakeLists.txt最后一行加入下面两行代码, 让编译器编译wang2.cpp这个文件，不然不会主动编译
                    add_executable(wang2_node src/wang2_pop.cpp)
                    ament_target_dependencies(wang2_node rclcpp)
                C++比Python要麻烦的地方在于，需要手动将编译好的文件安装到install/village_wang/lib/village_wang下，否则source之后依然找不到
                    install(TARGETS
                        wang2_node
                        DESTINATION lib/${PROJECT_NAME}
                    )
        7. 使用 colcon build 在工作空间下进行编译
            由于我们只需要编译village_wang这个功能包，所以可以使用:
                colcon build --packages-select village_wang
        8. source install/setup.bash
        9. 查看新建的功能包是否存在 ros2 pkg list | grep village
        10. ros2 run village_wang wang2_node
        11. 查看是否存在wang2节点 ros2 node list
            查看打开的节点wang2的信息 ros2 node info /wang2
 */

int main(int argc, char *argv[])
{
    // 2. 初始化客户端库
    rclcpp::init(argc,argv);
    // 3. 新建节点对象
    auto node = std::make_shared<rclcpp::Node>("wang2");          // 创建指向rclcpp::Node的智能指针，传入参数为节点名称
    RCLCPP_INFO(node->get_logger(), "大家好，我是单身狗wang2.");     // 使用cpp打印日志的宏RCLCPP_INFO，传入节点的get_logger()和打印信息
    // 4. spin循环节点
    rclcpp::spin(node);
    // 5. 关闭客户端库
    rclcpp::shutdown();
    return 0;
}


