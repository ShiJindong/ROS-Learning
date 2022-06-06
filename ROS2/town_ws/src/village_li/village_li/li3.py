#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 导入服务接口
from village_interfaces.srv import BorrowMoney

""" 
    导入服务接口
    创建请求结果接收回调函数
    声明并创建客户端
    编写结果接收逻辑
    调用客户端发送请求
"""

class BaiPiaoNode(Node):
    """
    创建一个李三节点
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是李三，李四他哥，我可以白嫖李四的小说！")
        self.sub_ = self.create_subscription(String,"sexy_girl",self.recv_callback,10)
        # 声明并创建客户端
        self.borrow_client = self.create_client(BorrowMoney,"borrow_money")

    def recv_callback(self,novel):
        self.get_logger().info('李三: 我已经收到了: %s' % novel.data)

    def borrow_money_eat(self,money=10):
        """  
            调用客户端发送请求
        """
        self.get_logger().info("借钱吃麻辣烫了，要借%d元" %money)
        # 确认服务是否在线,若在线则跳出循环，否则每等1秒，发出一条warn
        while not self.borrow_client.wait_for_service(1.0):
            self.get_logger().warn("服务不在线，我再等等...")
        # 构造请求内容
        request = BorrowMoney.Request()
        request.name = self.get_name()    # 获得当前节点的名称，作为发送请求的name属性
        request.money = money
        # 发送异步借钱请求
        self.borrow_client.call_async(request).add_done_callback(self.borrow_response_callback)
        """  
            以上这条异步环状服务请求和应答处理语句环环相扣:
                step 1. li3作为客户端borrow_client发送创建的请求request
                step 2. li4作为服务端borrow_server接收li3的request后, 调用回调函数borrow_money_callback来处理request,生成response
                step 3. li3的客户端borrow_client接受到响应response后, 调用回调函数borrow_response_callback来处理response
            由于客户端在得到响应之前,仍然可以发送其他请求,不需要等待,所以使用异步的服务发起方式call_async
        """
        
        

    def borrow_response_callback(self,response):
        """ 
            借钱结果回调
            编写结果接收逻辑
        """
        result = response.result()
        if result.success:
            self.get_logger().info("借到 %d元钱，去吃麻辣烫了!" %result.money)
        else:
            self.get_logger().info("连几块钱都不给，真抠门...")


def main(args=None):
    """
    ros2运行该节点的入口函数，可配置函数名称
    """
    rclpy.init(args=args) # 初始化rclpy
    node = BaiPiaoNode("li3")  # 新建一个节点
    node.borrow_money_eat()    # 调用借钱服务吃饭
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # rcl关闭