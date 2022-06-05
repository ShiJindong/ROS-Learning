import rclpy       # ros client library for python
from rclpy.node import Node   # 加载rclpy中的Node类，用于接下来创建节点

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

# 使用面向过程(POP)的思想来构建节点

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
                        "li4_node=village_li.li4:main"   
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
    # 2. 初始化客户端库
    rclpy.init(args=args)
    # 3. 新建节点对象
    li4_node = Node("li4")    # ros2 node list显示的节点名称

    li4_node.get_logger().info("大家好，我是作家li4.")   # 使用get_logger输出info日志

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