import os
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类
from launch.actions import DeclareLaunchArgument, LogInfo  # 添加日志输出（如果需要） 


def generate_launch_description():     # 自动生成launch文件的函数
    # 获取功能包路径
    rviz_config_path = '/home/vecow/real_car_experiment_ros2/src/CAN_decode/rviz/localizer_vis.rviz'
    return LaunchDescription([                  # 返回launch文件的描述信息
        # 启动自车定位解析节点
        Node(                                   # 配置一个节点的启动
            package='can_decode',                     # 节点所在的功能包
            executable='can_decode',          # 节点的可执行文件名
            name='can_decode',                # 对节点重新命名
            output='screen',
            parameters=[{
                'localization_custom_params.yaml': '/home/vecow/real_car_experiment_ros2/src/CAN_decode/config/localization_custom_params.yaml'
            }]
        ),

        # 启动可视化节点
        Node(
            package='can_decode',  # 可视化节点所在的功能包
            executable='ins_visualizer',  # 可视化节点的可执行文件名
            name='ins_visualizer',  # 节点名称
            output='screen',
            parameters=[{
                'localization_custom_params.yaml': '/home/vecow/real_car_experiment_ros2/src/CAN_decode/config/localization_custom_params.yaml'
            }]
        ),

        # 启动 RViz 并加载指定的 .rviz 配置文件
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])