import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('laplace_pathfinder')
    scenario_config_path = os.path.join(pkg_share_dir, 'config', 'scenario_config.yaml')

    config_defaults = {
        'model': 'burger',
        'use_sim_time': 'true',
        'world': 'dynamic_logistics_warehouse',
        'x_pose': '0.0',
        'y_pose': '0.0',
        'z_pose': '0.0',
        'yaw': '0.0',
        'goal_x_pose': '2.0',
        'goal_y_pose': '2.0',
        'goal_z_pose': '0.0',
        'goal_yaw': '0.0',
    }

    if os.path.exists(scenario_config_path):
        with open(scenario_config_path, 'r', encoding='utf-8') as config_file:
            scenario_config = yaml.safe_load(config_file) or {}

        turtlebot3_config = scenario_config.get('entities', {}).get('turtlebot3', {})
        simulation_config = scenario_config.get('simulation', {})
        start_pose = turtlebot3_config.get('start_pose', {})
        goal_pose = turtlebot3_config.get('goal_pose', {})

        config_defaults['model'] = str(turtlebot3_config.get('model', config_defaults['model']))
        config_defaults['use_sim_time'] = str(
            simulation_config.get('use_sim_time', config_defaults['use_sim_time'])
        ).lower()
        config_defaults['world'] = str(simulation_config.get('world', config_defaults['world']))
        config_defaults['x_pose'] = str(start_pose.get('x', config_defaults['x_pose']))
        config_defaults['y_pose'] = str(start_pose.get('y', config_defaults['y_pose']))
        config_defaults['z_pose'] = str(start_pose.get('z', config_defaults['z_pose']))
        config_defaults['yaw'] = str(start_pose.get('yaw', config_defaults['yaw']))
        config_defaults['goal_x_pose'] = str(goal_pose.get('x', config_defaults['goal_x_pose']))
        config_defaults['goal_y_pose'] = str(goal_pose.get('y', config_defaults['goal_y_pose']))
        config_defaults['goal_z_pose'] = str(goal_pose.get('z', config_defaults['goal_z_pose']))
        config_defaults['goal_yaw'] = str(goal_pose.get('yaw', config_defaults['goal_yaw']))

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    goal_x_pose = LaunchConfiguration('goal_x_pose')
    goal_y_pose = LaunchConfiguration('goal_y_pose')
    goal_z_pose = LaunchConfiguration('goal_z_pose')
    goal_yaw = LaunchConfiguration('goal_yaw')

    start_marker_model = os.path.join(pkg_share_dir, 'models', 'start_marker.sdf')
    goal_marker_model = os.path.join(pkg_share_dir, 'models', 'goal_marker.sdf')

    selected_world = config_defaults['world']
    if selected_world == 'dynamic_logistics_warehouse':
        world_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('dynamic_logistics_warehouse'),
                    'launch',
                    'logistics_warehouse.launch.py'
                )
            ),
            launch_arguments={
                'gui': gui,
                'verbose': verbose,
                'paused': paused,
                'use_sim_time': use_sim_time,
                'headless': headless,
            }.items()
        )
    elif selected_world == 'aws_robomaker_bookstore_world':
        world_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('aws_robomaker_bookstore_world'),
                    'launch',
                    'bookstore.launch.py'
                )
            ),
            launch_arguments={
                'gui': gui,
            }.items()
        )
    else:
        raise ValueError(f'Unsupported simulation world: {selected_world}')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    spawn_turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw': yaw,
        }.items()
    )

    spawn_start_marker = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'start_marker',
            '-file', start_marker_model,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
    )

    spawn_goal_marker = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'goal_marker',
            '-file', goal_marker_model,
            '-x', goal_x_pose,
            '-y', goal_y_pose,
            '-z', goal_z_pose,
            '-Y', goal_yaw,
        ],
    )

    robot_nav_controller_params = os.path.join(
        pkg_share_dir, 'config', 'robot_nav_controller_params.yaml'
    )

    robot_nav_controller = Node(
        package='laplace_pathfinder',
        executable='robot_nav_controller',
        name='robot_nav_controller',
        output='screen',
        parameters=[
            robot_nav_controller_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    robot_nav_planner_params = os.path.join(
        pkg_share_dir, 'config', 'robot_nav_planner_params.yaml'
    )

    robot_nav_planner = Node(
        package='laplace_pathfinder',
        executable='robot_nav_planner',
        name='robot_nav_planner',
        output='screen',
        parameters=[
            robot_nav_planner_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=config_defaults['model']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value=config_defaults['use_sim_time']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x_pose', default_value=config_defaults['x_pose']),
        DeclareLaunchArgument('y_pose', default_value=config_defaults['y_pose']),
        DeclareLaunchArgument('z_pose', default_value=config_defaults['z_pose']),
        DeclareLaunchArgument('yaw', default_value=config_defaults['yaw']),
        DeclareLaunchArgument('goal_x_pose', default_value=config_defaults['goal_x_pose']),
        DeclareLaunchArgument('goal_y_pose', default_value=config_defaults['goal_y_pose']),
        DeclareLaunchArgument('goal_z_pose', default_value=config_defaults['goal_z_pose']),
        DeclareLaunchArgument('goal_yaw', default_value=config_defaults['goal_yaw']),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        world_launch,
        spawn_start_marker,
        spawn_goal_marker,
        robot_state_publisher_launch,
        spawn_turtlebot_launch,
        robot_nav_controller,
        robot_nav_planner,
    ])