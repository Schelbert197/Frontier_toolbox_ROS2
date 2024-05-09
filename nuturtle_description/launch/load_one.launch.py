# from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, \
    SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, \
    PathJoinSubstitution, EqualsSubstitution, TextSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Choose to use joint_state_publisher."),
        DeclareLaunchArgument(name="use_rviz",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Choose whether to launch RViz."),
        DeclareLaunchArgument(name="color",
                              default_value="purple",
                              choices=['red', 'green', 'blue', 'purple', ''],
                              description="Sets the color argument in the\
                            turtlebot3 urdf"),
        SetLaunchConfiguration(name="rviz_color",
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=LaunchConfiguration("color"),
            arguments=["-d", LaunchConfiguration("rviz_color")],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_rviz"), "true")),
            on_exit=Shutdown()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration("color"),
            parameters=[
                {"frame_prefix":
                    PathJoinSubstitution([(LaunchConfiguration('color')), '']),
                 "robot_description":
                    Command([TextSubstitution(text="xacro "),
                             PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"),
                         "urdf/turtlebot3_burger.urdf.xacro"]),
                        " color:=",
                        LaunchConfiguration("color")])}
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration("color"),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_jsp"), "true"))
        )
    ])
