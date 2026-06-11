from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "rto_simulation"

    world_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "worlds",
        "robotino_empty.sdf"
    ])

    robot_xacro = PathJoinSubstitution([
        FindPackageShare("rto_description"),
        "urdf",
        "robots",
        "rto-3.urdf.xacro"
    ])

    robot_description_content = Command([
        "xacro ",
        robot_xacro
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file]
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robotino",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05"
        ],
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen"
    )

    controller_node = Node(
        package="robotino_controller",
        executable="controller_pf",
        name="controller_pf",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,

        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        bridge,

        TimerAction(
            period=5.0,
            actions=[controller_node]
        ),
    ])