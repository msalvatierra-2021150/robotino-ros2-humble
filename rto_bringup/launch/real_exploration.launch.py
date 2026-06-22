from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_frontier = LaunchConfiguration("start_frontier")
    send_frontier_goal = LaunchConfiguration("send_frontier_goal")
    robot_ip = LaunchConfiguration("robot_ip")

    robot_xacro = PathJoinSubstitution([
        FindPackageShare("rto_description"),
        "urdf",
        "robots",
        "rto-3.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro]),
            value_type=str,
        ),
        "use_sim_time": use_sim_time,
    }

    mapper_params = PathJoinSubstitution([
        FindPackageShare("rto_simulation"),
        "config",
        "mapper_params_online_async.yaml",
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare("rto_simulation"),
        "config",
        "nav2_params_robotino.yaml",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Publishes /odom and odom -> base_link
    rto_odom = Node(
        package="rto_node",
        executable="rto_odometry_node",
        name="rto_odom",
        output="screen",
        parameters=[{"hostname": robot_ip}],
    )

    # Receives /cmd_vel and sends movement commands to Robotino
    rto_drive = Node(
        package="rto_node",
        executable="rto_node",
        name="rto_drive",
        output="screen",
        parameters=[{"hostname": robot_ip}],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ]),
        launch_arguments={
            "slam_params_file": mapper_params,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": autostart,
        }.items(),
    )

    frontier_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("frontier_exploration"),
                "launch",
                "classical_exploration.launch.py",
            ])
        ]),
        condition=IfCondition(start_frontier),
    )

    first_frontier_goal = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/frontier_pose",
            "frontier_interfaces/srv/FrontierGoal",
            "{goal_rank: 0}",
        ],
        output="screen",
        condition=IfCondition(send_frontier_goal),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("start_frontier", default_value="false"),
        DeclareLaunchArgument("send_frontier_goal", default_value="false"),

        # CHANGE THIS DEFAULT TO THE IP THAT WORKED MANUALLY
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.20"),

        robot_state_publisher,
        rto_odom,
        rto_drive,

        # Give odom + TF time to start before SLAM
        TimerAction(period=7.0, actions=[slam_toolbox]),

        # Give SLAM time to create /map before Nav2 starts
        TimerAction(period=30.0, actions=[nav2]),

        # Keep frontier off by default while debugging
        TimerAction(period=45.0, actions=[frontier_exploration]),
        TimerAction(period=55.0, actions=[first_frontier_goal]),
    ])