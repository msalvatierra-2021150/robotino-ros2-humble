from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_frontier = LaunchConfiguration("start_frontier")
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

    rto_odom = Node(
        package="rto_node",
        executable="rto_odometry_node",
        name="rto_odom",
        output="screen",
        parameters=[{"hostname": robot_ip}],
    )

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

    # This waits until /map publishes at least one message.
    # Only runs if start_frontier:=true.
    wait_for_map = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "echo '[launch] Waiting for first /map message before starting frontier...'; "
                "ros2 topic echo /map --once > /tmp/first_map_received.txt; "
                "echo '[launch] /map received. Starting frontier exploration...'"
            ),
        ],
        output="screen",
        condition=IfCondition(start_frontier),
    )

    # When wait_for_map exits successfully, start frontier.
    start_frontier_after_map = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map,
            on_exit=[
                TimerAction(period=5.0, actions=[frontier_exploration])
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("start_frontier", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.20"),

        robot_state_publisher,
        rto_odom,
        rto_drive,

        # Give odom + TF time to start before SLAM
        TimerAction(period=7.0, actions=[slam_toolbox]),

        # Give SLAM time to create /map before Nav2 starts
        TimerAction(period=30.0, actions=[nav2]),

        # Start waiting for /map after SLAM has had time to start.
        # Once /map is received, frontier starts automatically.
        TimerAction(period=50.0, actions=[wait_for_map]),

        start_frontier_after_map,
    ])