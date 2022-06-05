#include "rclcpp/rclcpp.hpp"
// 导入订阅者消息类型 std_msgs/msgs/String
#include "std_msgs/msg/string.hpp"
// 导入发布者消息类型 std_msgs/msgs/u_int32
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

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
         然后会自动打开C/C++ Configurations, 找到include path一栏，填入/opt/ros/foxy/
         然后就会生成c_cpp_properties.json文件，用来配置头文件路径
         就不会报错了
*/ 

// 使用面向对象(OOP)的思想来构建节点
//  定义 单身狗节点的类，该类继承自rclcpp::Node节点
class SingleDogNode : public rclcpp::Node
{
private:
    // 声明订阅者  (声明和定义分离，定义是在构造函数内)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_novel;
    // 声明发布者
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_money;

    // 创建回调函数
    void novel_callback(const std_msgs::msg::String::SharedPtr novels)
    {
        // 每收到新的一章小说，就发布稿费
        std_msgs::msg::UInt32 money;
        money.data = 10;
        pub_money->publish(money);

        // 编写消息处理逻辑
        RCLCPP_INFO(this->get_logger(),"朕已阅%s.",novels->data.c_str());

    }
public:
    SingleDogNode(std::string name):Node(name)   // 将单身狗类获得的参数name给到父类Node ？ 
    {
        RCLCPP_INFO(this->get_logger(),"大家好，我是单身狗%s.",name.c_str());    // 使用cpp打印日志的宏RCLCPP_INFO，传入节点的get_logger()和打印信息
        // 这里使用this指针来获取成员函数get_logger()
        // 注意: 这里传入的name是string类型的，而%s需要C风格的字符串，所以需要使用c_str()，否则会乱码

        // 创建订阅者
        // 由于成员函数作为回调函数必须先实例化，所以需要使用std::bind
        // std::bind(类的成员函数的指针，指向类的实例的指针，成员函数参数的占位符)
        sub_novel = this->create_subscription<std_msgs::msg::String>("sexy_girl",10,std::bind(&SingleDogNode::novel_callback,this,_1));
        // 假如SingleDogNode::novel_callback需要传入两个参数那么，需要用到两个占位符
        // sub_novel = this->create_subscription<std_msgs::msg::String>("sexy_girl",10,std::bind(&SingleDogNode::novel_callback,this,_1,_2));    
    
        // 创建发布者
        pub_money = this->create_publisher<std_msgs::msg::UInt32>("sexy_girl_money",10);
    }
};

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
                    add_executable(wang2_node src/wang2_oop.cpp)
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

/* 
    创建话题订阅者的一般流程：

        导入订阅的话题接口类型
        创建订阅回调函数
        声明并创建订阅者
        编写订阅回调处理逻辑 
*/

int main(int argc, char *argv[])
{
    // 2. 初始化客户端库
    rclcpp::init(argc,argv);
    // 3. 新建节点对象，使用事先定义好的单身狗类来创建对象
    auto node = std::make_shared<SingleDogNode>("wang2");          // 创建指向SingleDogNode的智能指针，传入参数为节点名称

    // 4. spin循环节点
    rclcpp::spin(node);
    // 5. 关闭客户端库
    rclcpp::shutdown();
    return 0;
}


