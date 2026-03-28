import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('laplace_pathfinder')
    robot_nav_controller_params = os.path.join(
        pkg_share_dir, 'config', 'robot_nav_controller_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_nav_controller_node = Node(
        package='laplace_pathfinder',
        executable='robot_nav_controller',
        name='robot_nav_controller',
        output='screen',
        parameters=[
            robot_nav_controller_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_nav_controller_node,
    ])
