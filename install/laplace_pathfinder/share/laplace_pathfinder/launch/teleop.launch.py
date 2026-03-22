import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('laplace_pathfinder'), 'config', 'scenario_config.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    model = config.get('entities', {}).get('turtlebot3', {}).get('model', 'burger')

    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        additional_env={'TURTLEBOT3_MODEL': model},
    )

    return LaunchDescription([teleop_node])
