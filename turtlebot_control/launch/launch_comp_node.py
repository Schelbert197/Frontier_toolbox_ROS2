from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the directory of the installed shared libraries
    turtlebot_control_lib_path = os.path.join(
        get_package_share_directory('turtlebot_control'),
        'lib', 'turtelbot_control'
    )

    # Get system's LD_LIBRARY_PATH from the current environment
    existing_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')

    # Set LD_LIBRARY_PATH dynamically to include the lib directory
    additional_env = {
        'LD_LIBRARY_PATH': f"{turtlebot_control_lib_path}:{existing_ld_library_path}"
    }

    container = ComposableNodeContainer(
        name='pointcloud_to_laserscan_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='turtlebot_control',
                plugin='PointCloudToLaserScan',
                name='pointcloud_to_laserscan',
                parameters=[
                    {'scan_height': 0.1},
                    {'range_min': 0.0},
                    {'range_max': 100.0},
                    {'scan_frame': 'base_link'},
                    {'scan_topic': '/scan'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
        additional_env=additional_env # Setting env variable
    )

    return LaunchDescription([container])