import rclpy       # ros client library for python
from rclpy.node import Node   # 加载rclpy中的Node类，用于接下来创建节点
from std_msgs.msg import String,UInt32  # 从标准消息类型中加载String类型 和 32位无符号整型
# 导入服务接口
from village_interfaces.srv import BorrowMoney

""" 
    创建工作空间
        mkdir -p town_ws/src
        cd town_ws/src
    创建python版本的功能包
        ros2 pkg create village_li --build-type ament_python --dependencies rclpy
            kg create 是创建包的意思
            --build-type 用来指定该包的编译类型，一共有三个可选项ament_python、ament_cmake、cmake
            --dependencies 指的是这个功能包的依赖，这里给了一个ros2的python客户端接口rclpy
            如果build-type什么都不写，ros2会默认类型为ament_cmake.
    创建节点文件
        在__init__.py同级别目录下创建一个叫做li4.py的文件

    注意: 智能提示需要先在vscode中安装python这个插件
"""

# 使用面向对象(OOP)的思想来构建节点
# 定义 作家节点的类，该类继承自Node节点
class WriterNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是作家%s." %name)   # 使用get_logger输出info日志

        # 13. 声明并创建发布者
        self.pub_novel = self.create_publisher(String,"sexy_girl",10)   # 参数: 消息类型，话题名，队列长度

        # 14. 编写发布逻辑发布数据
        self.count = 0
        # 定期发布小说
        self.timer_period = 5
        self.timer = self.create_timer(self.timer_period,self.timer_callback)   # 每5秒调用一次回调函数

        # 15. 声明并创建订阅者，这里用来收取稿费
        self.account = 80
        self.sub_money = self.create_subscription(UInt32,"sexy_girl_money",self.receive_money_callback,10)

        # 声明并创建服务端
        self.borrow_server = self.create_service(BorrowMoney,"borrow_money",self.borrow_money_callback)


    # 创建服务端回调函数
    def borrow_money_callback(self,request,responce):
        # 编写回调函数逻辑处理请求
        # request:
        # responce:
        self.get_logger().info("收到来自: %s的借钱请求,账户目前有: %d元" %(request.name,self.account))
        if request.money <= self.account*0.1:
            responce.success = True
            responce.money = request.money
            self.account -= request.money
            self.get_logger().info("借钱成功，借出 %d元, 目前账户还剩下 %d元" %(responce.money,self.account))
        else:
            responce.success = False
            responce.money = 0
            self.get_logger().info("对不起，大兄弟，最近手头紧，不能借给你!")
        return responce

    def timer_callback(self):
        msg = String()   
        # 由ros2 interface show std_msgs/msg/String可知String包含内容为data
        msg.data = "第%d回:潋滟湖 %d 次偶遇胡艳娘" % (self.count,self.count)  
        self.pub_novel.publish(msg)             # 让发布者发布消息
        self.get_logger().info("发布了一个章节的小说，内容是: %s" % msg.data)  
        self.count += 1

    def receive_money_callback(self,money):
        # 由ros2 interface show std_msgs/msg/UInt32可知String包含内容为data
        self.account += money.data
        self.get_logger().info("收到了%d元的稿费，现在账户里有%d元的钱." %(money.data,self.account))
        

def main(args=None):
    """ 
        ros2运行该节点的 入口函数
        编写ROS2节点的一般步骤
        1. 导入库文件
        2. 初始化客户端库
        3. 新建节点对象
        4. spin循环节点
        5. 关闭客户端库
        6. 在setup.py中加入配置，使得其可以找到该节点
                entry_points={
                    'console_scripts': [
                        "li4_node=village_li.li4_oop:main"   
                    ],
                },
            # 指明了: li4_node这个节点可以在village_li这个功能包的li4.py(.py可以省略)源文件内的入口函数main中找到
        7. 使用 colcon build 在工作空间下进行编译
            由于我们只需要编译village_li这个功能包，所以可以使用:
                colcon build --packages-select village_li
            如果我们不想每次改变python脚本内容后都去重新build一遍，我们可以使用
                colcon build --symlink-install
        8. source install/setup.bash
        9. 查看新建的功能包是否存在 ros2 pkg list | grep village
        10. ros2 run village_li li4_node
        11. 查看打开的节点li4的信息 ros2 node info /li4
    """

    """ 
        消息发布: 
            12. 导入消息类型
            13. 声明并创建发布者
            14. 编写发布逻辑发布数据
            15. 声明并创建订阅者，这里用来收取稿费
            16. 可通过terminal输入以下命令进行付款
                ros2 topic pub /sexy_girl_money std_msgs/msg/UInt32 "{data: 10}" -1
    """

    """ 
        服务端:
            导入服务接口
            创建服务端回调函数
            声明并创建服务端
            编写回调函数逻辑处理请求
    """

    # 2. 初始化客户端库
    rclpy.init(args=args)
    # 3. 新建节点对象 (使用定义好的WriterNode类来定义新的对象)
    li4_node = WriterNode("li4")    # ros2 node list显示的节点名称

    # 4. spin循环节点
    rclpy.spin(li4_node)      #将该节点放入回旋中，让其不会退出
    # 5. 关闭客户端库
    rclpy.shutdown()

    # 6. 在setup.py中加入配置，使得其可以找到该节点

    # 7. 使用 colcon build 在工作空间下进行编译

    # 8. source install/setup.bash
    # 9. 查看新建的功能包是否存在 ros2 pkg list | grep village
    # 10. ros2 run village_li li4_node
    # 11. 查看打开的节点li4的信息 ros2 node info /li4