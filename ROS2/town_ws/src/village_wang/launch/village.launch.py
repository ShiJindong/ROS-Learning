# 1. 导入头文件
from launch import LaunchDescription        # 用于对launch文件内容进行描述
from launch_ros.actions import Node         # 一个是Node，用于声明节点所在的位置

# 2. 定义函数名称为：generate_launch_description, ROS2会对该函数名字做识别
def generate_launch_description():
    # 3. 创建节点描述 (这个Node是launch_ros.actions内定义的Node)
    # 创建Actions.Node对象li_node，标明李四所在位置
    # 当我们启动节点时，可以对该节点的参数进行赋值，比如我们可以尝试在启动节点时修改李四写书的速度，改变王二卖书的价格
    # 我们可以修改命名空间，这样就可以区分两个同名节点
    li4_node = Node(
        package="village_li",
        namespace="mirror_town",
        executable="li4_node",
        parameters=[{'writer_timer_period':1}]    
        )
    # 创建Actions.Node对象wang2_node，标明王二所在位置
    wang2_node = Node(
        package="village_wang",
        namespace="mirror_town",
        executable="wang2_node",
        parameters=[{'novel_price':1}]
    )

    # 4. 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([li4_node, wang2_node])
    # 5. 返回让ROS2根据launch描述执行节点
    return launch_description

    # 6. ament_cmake编译模式下: 编辑CMakeLists.txt文件，将launch目录下的所有.launch.py文件拷贝到目录install/village_wang/share/village_wang/launch下
    """  
    install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
    """

        