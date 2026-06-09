from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hostname = LaunchConfiguration("hostname")

    bringup_launch = PathJoinSubstitution([
        FindPackageShare("rto_bringup"),
        "launch",
        "rto_bringup_launch.py"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("rto_bringup"),
        "rviz",
        "robotino_odom_sensors.rviz"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "hostname",
            default_value="172.27.1.1",
            description="Robotino IP address"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "hostname": hostname
            }.items()
        ),

        Node(
            package="rto_node",
            executable="rto_odometry_node",
            name="rto_odometry_node",
            output="screen",
            parameters=[
                {"hostname": hostname}
            ]
        ),

        Node(
            package="robotino_experiments",
            executable="obstacle_stop",
            name="obstacle_stop",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config]
        ),
    ])