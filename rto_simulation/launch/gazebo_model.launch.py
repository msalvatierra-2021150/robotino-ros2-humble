from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "rto_simulation"
    pkg_share = FindPackageShare(pkg_name)

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    apriltag_config = LaunchConfiguration("apriltag_config")
    use_saved_map = LaunchConfiguration("use_saved_map")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    declare_apriltag_config = DeclareLaunchArgument(
        "apriltag_config",
        default_value=(
            "/home/mike/robotino3/robotino3ros2/config/"
            "apriltag_36h11.yaml"
        ),
        description="Path to the AprilTag ROS configuration YAML file",
    )

    declare_use_saved_map = DeclareLaunchArgument(
        "use_saved_map",
        default_value="false",
        description=(
            "If true, load a saved map and use AMCL. "
            "If false, run SLAM Toolbox and frontier exploration."
        ),
    )

    declare_map_yaml_file = DeclareLaunchArgument(
        "map_yaml_file",
        default_value="/home/mike/robotino_maps/robotino_lab.yaml",
        description="Absolute path to the saved occupancy-map YAML file",
    )

    # ------------------------------------------------------------------
    # Package files
    # ------------------------------------------------------------------
    world_file = PathJoinSubstitution(
        [pkg_share, "worlds", "arena_5_5x3.sdf"]
    )

    bridge_config = PathJoinSubstitution(
        [pkg_share, "config", "gz_bridge.yaml"]
    )

    robot_state_publisher_config = PathJoinSubstitution(
        [pkg_share, "config", "robot_state_publisher.yaml"]
    )

    nav2_params_file = PathJoinSubstitution(
        [pkg_share, "config", "nav2_params_robotino.yaml"]
    )

    slam_params_file = PathJoinSubstitution(
        [pkg_share, "config", "mapper_params_online_async.yaml"]
    )

    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("rto_description"),
            "urdf",
            "robots",
            "rto-3.urdf.xacro",
        ]
    )

    robot_description_content = Command(["xacro ", robot_xacro])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # ------------------------------------------------------------------
    # Gazebo resource paths
    # ------------------------------------------------------------------
    set_gz_worlds_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_share, "worlds"]),
    )

    set_gz_models_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_share, "models"]),
    )

    set_ign_worlds_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_share, "worlds"]),
    )

    set_ign_models_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_share, "models"]),
    )

    # ------------------------------------------------------------------
    # Gazebo and robot
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            robot_description,
            robot_state_publisher_config,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robotino",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            apriltag_config,
        ],
        remappings=[
            ("image_rect", "/camera/image_rect"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    image_rectify_node = Node(
        package="image_proc",
        executable="rectify_node",
        name="image_rectify_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("image_rect", "/camera/image_rect"),
        ],
    )

    # ------------------------------------------------------------------
    # Mapping mode
    # use_saved_map:=false
    #
    # SLAM Toolbox owns /map and map -> odom.
    # Frontier exploration is available only in this mode.
    # ------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": "true",
        }.items(),
        condition=UnlessCondition(use_saved_map),
    )

    frontier_detector = Node(
        package="frontier_exploration",
        executable="classical_frontier_detector",
        name="classical_frontier_detector",
        output="screen",
        parameters=[
            {
                "region_size_thresh": 10,
                "robot_width": 0.350,
                "occupancy_map_msg": "map",
                "use_sim_time": True,
            }
        ],
        condition=UnlessCondition(use_saved_map),
    )

    frontier_explorer = Node(
        package="frontier_exploration",
        executable="frontier_exploration_node.py",
        name="frontier_exploration_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_saved_map),
    )

    # ------------------------------------------------------------------
    # Saved-map localization mode
    # use_saved_map:=true
    #
    # Map Server publishes the saved map and AMCL owns map -> odom.
    # Do not run SLAM Toolbox at the same time.
    # ------------------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "localization_launch.py",
                ]
            )
        ),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": "true",
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
        condition=IfCondition(use_saved_map),
    )

    # Navigation is needed in both mapping and saved-map modes.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_apriltag_config,
            declare_use_saved_map,
            declare_map_yaml_file,
            set_gz_worlds_path,
            set_gz_models_path,
            set_ign_worlds_path,
            set_ign_models_path,
            gazebo,
            robot_state_publisher,
            apriltag_node,
            TimerAction(
                period=3.0,
                actions=[spawn_robot],
            ),
            bridge,
            camera_image_bridge,
            image_rectify_node,
            # Exactly one localization source starts here:
            # SLAM Toolbox when false, AMCL + Map Server when true.
            TimerAction(
                period=5.0,
                actions=[slam_launch],
            ),
            TimerAction(
                period=5.0,
                actions=[localization_launch],
            ),
            TimerAction(
                period=8.0,
                actions=[nav2_launch],
            ),
            # These are skipped automatically in saved-map mode.
            TimerAction(
                period=12.0,
                actions=[frontier_explorer],
            ),
            TimerAction(
                period=12.0,
                actions=[frontier_detector],
            ),
        ]
    )