# code style 1(from lidar_slam) 
import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    map_generator_param = launch.substitutions.LaunchConfiguration(
        'map_generator_param',
        default=os.path.join(
            get_package_share_directory('adas'),
            'config',
            'control.param.yaml'))

    node1 = launch_ros.actions.Node(
        package='adas',
        executable='map_generator',
        name='map_generator',
        parameters=[map_generator_param],
        output='screen'
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map_generator_param',
            default_value=map_generator_param,
            description='map_generator param load'),
            node1,
            ])