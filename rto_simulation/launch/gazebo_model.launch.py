from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "rto_simulation"
    pkg_share = FindPackageShare(pkg_name)

    apriltag_config = LaunchConfiguration("apriltag_config")

    declare_apriltag_config = DeclareLaunchArgument(
        "apriltag_config",
        default_value="/home/mike/robotino3/robotino3ros2/config/apriltag_36h11.yaml",
        description="Path to the AprilTag ROS configuration YAML file"
    )

    world_file = PathJoinSubstitution([
        pkg_share,
        "worlds",
        "arena_5_5x3.sdf"
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

    set_gz_worlds_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "worlds"
        ])
    )

    set_gz_models_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "models"
        ])
    )

    set_ign_worlds_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "worlds"
        ])
    )

    set_ign_models_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([
            pkg_share,
            "models"
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

    camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image_raw"
        ],
        output="screen"
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            apriltag_config
        ],
        remappings=[
            ('image_rect', '/camera/image_rect'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    image_rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='image_rectify_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('image_rect', '/camera/image_rect'),
        ]
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

    frontier_detector = Node(
        package='frontier_exploration',
        executable='classical_frontier_detector',
        name='classical_frontier_detector',
        output='screen',
        parameters=[{
            'region_size_thresh': 10,
            'robot_width': 0.350,
            'occupancy_map_msg': 'map',
        }]
    )

    frontier_explorer = Node(
        package="frontier_exploration",
        executable="frontier_exploration_node.py",
        name="frontier_exploration_node",
        output="screen",
    )

    return LaunchDescription([
        declare_apriltag_config,
        set_gz_worlds_path,
        set_gz_models_path,
        set_ign_worlds_path,
        set_ign_models_path,

        gazebo,
        robot_state_publisher,
        apriltag_node,

        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        bridge,
        camera_image_bridge,
        image_rectify_node,

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
            actions=[frontier_explorer]
        ),

        TimerAction(
            period=12.0,
            actions=[frontier_detector]
        )
    ])