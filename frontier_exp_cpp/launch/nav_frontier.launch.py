import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time if true')

    # Get the package path
    pkg_dir = FindPackageShare('frontier_exp_cpp').find('frontier_exp_cpp')

    # Define parameter file based on simulation or real robot
    simulation_params_file = os.path.join(
        pkg_dir, 'config', 'simulation_params.yaml')
    real_robot_params_file = os.path.join(
        pkg_dir, 'config', 'real_robot_params.yaml')

    # Use the correct params file based on 'use_sim_time'
    params_file = LaunchConfiguration('params_file')
    conditional_params_file = IfCondition(LaunchConfiguration('use_sim_time'),
                                          then_value=simulation_params_file,
                                          else_value=real_robot_params_file)

    # Include the correct params file based on use_sim_time
    param_substitution = RewrittenYaml(
        source_file=conditional_params_file,
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True)

    # Define your lifecycle node
    frontier_node = LifecycleNode(
        package='frontier_exp_cpp',
        executable='frontier_lc',
        name='frontier_explorer',
        namespace='',
        output='screen',
        parameters=[param_substitution],
        remappings=[('/map', '/map'), ('/goal_pose', '/goal_pose')],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Handle Nav2 lifecycle nodes
    nav2_launch_file = FindPackageShare('nav2_bringup').find(
        'nav2_bringup') + '/launch/navigation_launch.py'

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,

        # Choose the params file depending on simulation
        DeclareLaunchArgument(
            'params_file', default_value=conditional_params_file),

        # Log which file is being used
        LogInfo(msg=["Using params file: ",
                LaunchConfiguration('params_file')]),

        # Include Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file')
            }.items()
        ),

        # Launch your lifecycle node
        frontier_node
    ])
