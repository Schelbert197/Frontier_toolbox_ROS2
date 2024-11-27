import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Declare a launch argument for sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (true) or real robot time (false)'
    )

    # Get the path to the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get the path to the nav2_params.yaml in your package's launch folder
    package_share_dir = get_package_share_directory('frontier_exp_cpp')
    nav2_params_file_robot = os.path.join(
        package_share_dir, 'config', 'nav2_params_robot.yaml')
    nav2_params_file_sim = os.path.join(
        package_share_dir, 'config', 'nav2_params_sim.yaml')
    frontier_params_file = os.path.join(
        get_package_share_directory('frontier_exp_cpp'),
        'config',
        'frontier_params.yaml'
    )

    # Function to conditionally include the params file if use_sim_time is false
    def include_nav2_with_params(context):
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        nav2_launch_arguments = {'use_sim_time': use_sim_time}.items()

        # Add the params file only if use_sim_time is false (real robot)
        if use_sim_time == 'false':
            nav2_launch_arguments = {
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file_robot
            }.items()
        else:
            nav2_launch_arguments = {
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file_sim
            }.items()

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch',
                             'navigation_launch.py')
            ),
            launch_arguments=nav2_launch_arguments
        )]

    # Define the frontier exploration node
    frontier_node = LifecycleNode(
        package='frontier_exp_cpp',
        executable='frontier_lc',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    frontier_params_file],
        namespace=''
    )

    nav2_client_node = Node(
        package='nav_client_cpp',
        executable='nav_node',
        name='nav_to_pose',
        output='screen',
        parameters=[frontier_params_file])

    return LaunchDescription([
        # Declare sim_time and viewpoint depth arguments
        use_sim_time_arg,

        # Conditionally launch Nav2 with or without params file
        OpaqueFunction(function=include_nav2_with_params),

        # Launch nav2 client node
        nav2_client_node,

        # Launch frontier node
        frontier_node,
    ])
