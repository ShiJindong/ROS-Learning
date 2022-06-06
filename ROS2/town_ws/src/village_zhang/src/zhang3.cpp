#include "rclcpp/rclcpp.hpp"
// 1. 导入服务接口
#include "village_interfaces/srv/sell_novel.hpp"

// 提前声明的占位符，留着创建客户端的时候用
using std::placeholders::_1;

/*
    创建一个类节点，名字叫做PoorManNode,继承自Node.
*/
class PoorManNode : public rclcpp::Node
{

public:
    /* 构造函数 */
    PoorManNode(std::string name) : Node(name)
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "大家好，我是得了穷病的张三.");
        // 3. 声明并创建客户端
        novel_client = this->create_client<village_interfaces::srv::SellNovel>("sell_novel");
    }

    // 5. 调用客户端发送请求
    void buy_novels(int money=3)
    {
        RCLCPP_INFO(this->get_logger(),"准备去买二手小说了!");
        // 等待服务端上线
        while(!novel_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(),"等待服务端上线...");
        }
        // 构造请求数据
        auto request = std::make_shared<village_interfaces::srv::SellNovel_Request>();
        request->money = static_cast<uint32_t>(money);
        // 发送异步请求
        novel_client->async_send_request(request,std::bind(&PoorManNode::novels_callback,this,_1));
    }
private:
    rclcpp::Client<village_interfaces::srv::SellNovel>::SharedPtr novel_client;

    // 2. 创建请求结果接收回调函数，注意: 这里回调函数的参数是SharedFuture类型的响应response !!!
    void novels_callback(rclcpp::Client<village_interfaces::srv::SellNovel>::SharedFuture response)
    {
        // 4. 编写结果接收逻辑
        auto result = response.get();   // 指向response的指针
        RCLCPP_INFO(this->get_logger(),"收到了%d章小说,现在开始按章节读",result->novels.size());
        for(std::string novel:result->novels)
        {
            // 打印收到的每一章小说
            RCLCPP_INFO(this->get_logger(),"%s",novel.c_str());
        }
        RCLCPP_INFO(this->get_logger(),"接收到的小说读完了，真精彩!");
    }
};

/* 
    编写ROS2服务通信客户端步骤:
        1. 导入服务接口
        2. 创建请求结果接收回调函数
        3. 声明并创建客户端
        4. 编写结果接收逻辑
        5. 调用客户端发送请求
 */

/*主函数*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个Zhang3的节点*/
    auto node = std::make_shared<PoorManNode>("zhang3");
    // 发送买二手小说的请求
    node->buy_novels(5);   // 花费5元

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}