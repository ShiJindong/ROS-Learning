#include "rclcpp/rclcpp.hpp"
// 导入订阅者消息类型 std_msgs/msgs/String
#include "std_msgs/msg/string.hpp"
// 导入发布者消息类型 std_msgs/msgs/u_int32
#include "std_msgs/msg/u_int32.hpp"
// 导入服务接口
// 在CmakeLists.txt和package.xml里面设置好interface对应的依赖后，
// 如果vscode还是找不到头文件，可以在vscode的include path里面添加${workspaceFolder}/install/village_interfaces/include/**
#include "village_interfaces/srv/sell_novel.hpp"
#include <queue>      // 用于存放二手书

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
    // 书库
    std::queue<std::string> novels_queue;     // 用于存放二手书的容器

    // ROS2中要使用多线程执行器和回调组来实现多线程，我们先在SingleDogNode中声明一个回调组
    rclcpp::CallbackGroup::SharedPtr sell_novels_callback_group;

    // 先声明服务端，后面会在构造函数中对其实例化
    rclcpp::Service<village_interfaces::srv::SellNovel>::SharedPtr sell_server;


    // 创建回调函数
    void novel_callback(const std_msgs::msg::String::SharedPtr novels)
    {
        // 每收到新的一章小说，就发布稿费
        std_msgs::msg::UInt32 money;
        money.data = 10;
        pub_money->publish(money);

        // 将新发布的书放入书库
        novels_queue.push(novels->data);

        // 编写消息处理逻辑
        RCLCPP_INFO(this->get_logger(),"朕已阅%s.",novels->data.c_str());
    }

    // 创建处理买书请求的回调函数
    void sell_novel_callback(const village_interfaces::srv::SellNovel::Request::SharedPtr request,
                             const village_interfaces::srv::SellNovel::Response::SharedPtr response)
    {
        /* 
            死锁:
                判断当前的书章节数量够不够，不够就需要攒书，再返回
                等待novels_queue书库里的书的数量达到要卖出去的
                但是等待会让当前的线程阻塞，这样一来便无法通过调用回调函数novel_callback来增加书的数量，进一步造成无法退出当前线程
                两个回调函数sell_novel_callback和novel_callback都在等待对方结束，故而造成了死锁 !!!
                原因是ROS2默认是单线程的，同时只有一个线程在跑，大家都是顺序执行，你干完我干，一条线下去。
                所以为了解决这个问题，我们可以使用多线程，即每次收到服务请求后，单独开一个线程来处理，不影响其他部分
            -->   解决死锁:
                    1. ROS2中要使用多线程执行器和回调组来实现多线程，我们需要先在SingleDogNode中声明一个回调组，在构造函数创建服务端时，将该回调组传进去
                    2. 同时需要在main函数中定义多线程执行器，来实现多线程
         */
        RCLCPP_INFO(this->get_logger(),"受到一个买书的请求,一共给了%d元",request->money);
        // 计算应该返回给客户端的小说的数量
        unsigned int num = static_cast<int>(request->money/(1.0));

        if(num > novels_queue.size())
        {
            // 等待凑齐书
            RCLCPP_INFO(this->get_logger(),"书库书不够，因为书库里只有%d本书,而要卖出%d本书!",novels_queue.size(),num);

            rclcpp::Rate rate(1);

            while(novels_queue.size() < num)
            {
                RCLCPP_INFO(this->get_logger(),"等待中，目前还差%d本小说!",num-novels_queue.size());
                rate.sleep();    // 休眠1秒，执行一次
            }
        }

        RCLCPP_INFO(this->get_logger(),"当前书库里的书有%d本,大于要卖出去的书的数量%d",novels_queue.size(),num);

        for(int i=0;i<static_cast<int>(num);++i)
        {
            response->novels.push_back(novels_queue.front());   // 将书库头部的string压入novels
            // 问: 服务接口内response的变量string[] novels定义为数组而非容器，为什么可以用push_back呢???
            // 答: ROS2会将数组string[] novels转化为std::vector<std:.string>，存放于头文件中，所以可以使用push_back()
            novels_queue.pop();  // 由于队列queue先进先出，故pop的是头部的string
        }

        RCLCPP_INFO(this->get_logger(),"已经成功卖出%d本小说,收入%d元!",response->novels.size(),request->money);

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

        // 实例化回调组,只需要传入一个参数，那就是回调组的类型MutuallyExclusive
        sell_novels_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 实例化服务端
        /* 
            这个实例化服务端的代码比较复杂:
                参数:   
                    1. 服务名 
                    2. 构造函数经bind实例化后成为的回调函数（由于sell_novel_callback需要传入两个参数，所以需要两个占位符） 
                    3. 由于需要使用第四个参数，故不可省略第三个参数，传入默认值即可
                    4. 实例化后的回调组
         */
        sell_server = this->create_service<village_interfaces::srv::SellNovel>(
                        "sell_novel",
                        std::bind(&SingleDogNode::sell_novel_callback,this,_1,_2),
                        rmw_qos_profile_services_default,
                        sell_novels_callback_group);
                    // 在创建服务端时使用回调组的好处是:
                    // 一旦接受到sell_novel的服务请求后，会单独开出线程，在回调组内执行回调函数sell_novel_callback，
                    // 而不影响原线程在接受到新的sexy_girl消息后，执行回调函数novel_callback
        
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

/* 
    创建C++服务通信服务端的步骤:
        导入服务接口
        创建服务端回调函数
        声明并创建服务端
        编写回调函数逻辑处理请求
 */

int main(int argc, char *argv[])
{
    // 2. 初始化客户端库
    rclcpp::init(argc,argv);
    
    // 3. 新建节点对象，使用事先定义好的单身狗类来创建对象
    auto node = std::make_shared<SingleDogNode>("wang2");       // 创建指向SingleDogNode的智能指针，传入参数为节点名称

    rclcpp::executors::MultiThreadedExecutor executor;          // 多线程执行器
    executor.add_node(node);
    executor.spin();      // 将多线程执行器加入回旋，而不是节点node加入回旋

    // 4. spin循环节点
    // rclcpp::spin(node);

    // 5. 关闭客户端库
    rclcpp::shutdown();
    return 0;
}


