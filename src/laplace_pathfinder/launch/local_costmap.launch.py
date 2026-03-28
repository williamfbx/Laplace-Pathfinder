import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('laplace_pathfinder')
    costmap_params = os.path.join(pkg_share_dir, 'config', 'local_costmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='costmap',
        output='screen',
        parameters=[
            costmap_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('costmap', '/local_costmap'),
            ('costmap_updates', '/local_costmap_updates'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['costmap/costmap']},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        local_costmap_node,
        lifecycle_manager,
    ])
