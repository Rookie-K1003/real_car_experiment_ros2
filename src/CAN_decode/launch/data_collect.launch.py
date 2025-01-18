# code style 2(from guyuehome) 
import os
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类


def generate_launch_description():     # 自动生成launch文件的函数
   

    return LaunchDescription([                  # 返回launch文件的描述信息
        Node(                                   # 配置一个节点的启动
            package='can_decode',                     # 节点所在的功能包
            executable='can_decode',          # 节点的可执行文件名
            name='can_decode',                # 对节点重新命名
        )
    ])