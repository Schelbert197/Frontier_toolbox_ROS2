import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Declare a launch argument for sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (true) or real robot time (false)'
    )

    # Declare a launch argument for viewpoint_depth
    viewpoint_depth_arg = DeclareLaunchArgument(
        'viewpoint_depth',
        default_value='1.0',  # Default value
        description='Minimize frontier distance to X[m] in front of robot'
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
    frontier_node = LifecycleNode(
        package='frontier_exp',
        executable='frontier_lc',
        name='frontier_exploration',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'viewpoint_depth': LaunchConfiguration('viewpoint_depth')}],
        namespace=''
    )

    # Event handler to activate the frontier node after Nav2 becomes active
    frontier_activation_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=frontier_node,  # Frontier node is the lifecycle node
            goal_state='active',  # Transition to 'active' when Nav2 becomes active
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=frontier_node,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        # Declare sim_time and viewpoint depth arguments
        use_sim_time_arg,
        viewpoint_depth_arg,

        # Launch Nav2
        nav2_launch,

        # Launch frontier node
        frontier_node,

        # Event handler to start the frontier node after Nav2 is active
        frontier_activation_event_handler,
    ])
