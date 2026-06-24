from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "rto_simulation"
    pkg_share = FindPackageShare(pkg_name)

    world_file = PathJoinSubstitution([
        pkg_share,
        "worlds",
        "robotino_slam_practice_arena.sdf"
    ])

    bridge_config = PathJoinSubstitution([
        pkg_share,
        "config",
        "gz_bridge.yaml"
    ])

    robot_state_publisher_config = PathJoinSubstitution([
        pkg_share,
        "config",
        "robot_state_publisher.yaml"
    ])

    nav2_params_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "nav2_params_robotino.yaml"
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

    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "worlds"
        ])
    )

    set_ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "worlds"
        ])
    )

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
        name="robot_state_publisher",
        parameters=[
            robot_description,
            robot_state_publisher_config
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
        parameters=[
            {
                "config_file": bridge_config
            }
        ],
        output="screen"
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution([
                pkg_share,
                "config",
                "mapper_params_online_async.yaml"
            ]),
            "use_sim_time": "true"
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file
        }.items()
    )

    frontier_exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("frontier_exploration"),
                "launch",
                "classical_exploration.launch.py"
            ])
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,

        gazebo,
        robot_state_publisher,

        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        bridge,

        TimerAction(
            period=5.0,
            actions=[slam_launch]
        ),

        TimerAction(
            period=8.0,
            actions=[nav2_launch]
        ),

        TimerAction(
            period=12.0,
            actions=[frontier_exploration_launch]
        )
    ])