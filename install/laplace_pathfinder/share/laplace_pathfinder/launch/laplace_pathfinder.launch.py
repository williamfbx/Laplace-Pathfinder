import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('laplace_pathfinder'), 'launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_nav_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_nav_planner.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_nav_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_nav_controller.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        simulation_launch,
        robot_nav_planner_launch,
        robot_nav_controller_launch,
    ])