import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument for sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (true) or real robot time (false)'
    )

    # Get the path to the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Include Nav2 launch description
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration(
            'use_sim_time')}.items(),
    )

    # Define the frontier exploration node
    frontier_node = Node(
        package='frontier_exp',
        executable='frontier',
        name='frontier_exploration',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        # Declare sim_time argument
        use_sim_time_arg,

        # Launch Nav2
        nav2_launch,

        # Launch frontier exploration node
        frontier_node,
    ])
