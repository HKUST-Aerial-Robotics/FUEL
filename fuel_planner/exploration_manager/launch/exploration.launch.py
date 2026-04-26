from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")
    odom_topic = LaunchConfiguration("odom_topic")

    algorithm_launch = PathJoinSubstitution(
        [FindPackageShare("exploration_manager"), "launch", "algorithm.launch.py"]
    )
    simulator_launch = PathJoinSubstitution(
        [FindPackageShare("exploration_manager"), "launch", "simulator.launch.py"]
    )

    declared_arguments = [
        DeclareLaunchArgument("map_size_x", default_value="50.0"),
        DeclareLaunchArgument("map_size_y", default_value="50.0"),
        DeclareLaunchArgument("map_size_z", default_value="10.0"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="1.0"),
        DeclareLaunchArgument("odom_topic", default_value="/state_ukf/odom"),
    ]

    algorithm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(algorithm_launch),
        launch_arguments={
            "map_size_x_": map_size_x,
            "map_size_y_": map_size_y,
            "map_size_z_": map_size_z,
            "box_min_x": "-10.0",
            "box_min_y": "-15.0",
            "box_min_z": "0.0",
            "box_max_x": "10.0",
            "box_max_y": "15.0",
            "box_max_z": "2.0",
            "odometry_topic": odom_topic,
            "sensor_pose_topic": "/pcl_render_node/sensor_pose",
            "depth_topic": "/pcl_render_node/depth",
            "cloud_topic": "/pcl_render_node/cloud",
            "cx": "321.04638671875",
            "cy": "243.44969177246094",
            "fx": "387.229248046875",
            "fy": "387.229248046875",
            "max_vel": "2.0",
            "max_acc": "2.0",
        }.items(),
    )

    traj_server = Node(
        package="plan_manage",
        executable="traj_server",
        name="traj_server",
        output="screen",
        remappings=[
            ("/position_cmd", "planning/pos_cmd"),
            ("/odom_world", odom_topic),
        ],
        parameters=[
            {
                "traj_server.time_forward": 1.5,
                "traj_server.pub_traj_id": 4,
                "traj_server.init_x": init_x,
                "traj_server.init_y": init_y,
                "traj_server.init_z": init_z,
                "perception_utils.top_angle": 0.56125,
                "perception_utils.left_angle": 0.69222,
                "perception_utils.right_angle": 0.68901,
                "perception_utils.max_dist": 4.5,
                "perception_utils.vis_dist": 1.0,
            }
        ],
    )

    waypoint_generator = Node(
        package="waypoint_generator",
        executable="waypoint_generator",
        name="waypoint_generator",
        output="screen",
        remappings=[
            ("~/odom", odom_topic),
            ("~/goal", "/move_base_simple/goal"),
            ("~/traj_start_trigger", "/traj_start_trigger"),
        ],
        parameters=[{"waypoint_type": "point"}],
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulator_launch),
        launch_arguments={
            "map_size_x_": map_size_x,
            "map_size_y_": map_size_y,
            "map_size_z_": map_size_z,
            "init_x": init_x,
            "init_y": init_y,
            "init_z": init_z,
            "odometry_topic": odom_topic,
            "c_num": "0",
            "p_num": "130",
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [algorithm, traj_server, waypoint_generator, simulator]
    )
