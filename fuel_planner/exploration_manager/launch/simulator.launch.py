from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("init_x"),
        DeclareLaunchArgument("init_y"),
        DeclareLaunchArgument("init_z"),
        DeclareLaunchArgument("map_size_x_"),
        DeclareLaunchArgument("map_size_y_"),
        DeclareLaunchArgument("map_size_z_"),
        DeclareLaunchArgument("box_min_x", default_value="-10.0"),
        DeclareLaunchArgument("box_min_y", default_value="-15.0"),
        DeclareLaunchArgument("box_min_z", default_value="0.0"),
        DeclareLaunchArgument("box_max_x", default_value="10.0"),
        DeclareLaunchArgument("box_max_y", default_value="15.0"),
        DeclareLaunchArgument("box_max_z", default_value="2.0"),
        DeclareLaunchArgument("c_num", default_value="0"),
        DeclareLaunchArgument("p_num", default_value="130"),
        DeclareLaunchArgument("odometry_topic"),
        DeclareLaunchArgument("obj_num", default_value="1"),
    ]

    map_file = PathJoinSubstitution([FindPackageShare("map_generator"), "resource", "office.pcd"])

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="map_generator",
                executable="map_pub",
                name="map_pub",
                output="screen",
                arguments=[map_file],
            ),
            Node(
                package="so3_quadrotor_simulator",
                executable="quadrotor_simulator_so3",
                name="quadrotor_simulator_so3",
                output="screen",
                parameters=[
                    {
                        "rate.odom": 200.0,
                        "simulator.init_state_x": LaunchConfiguration("init_x"),
                        "simulator.init_state_y": LaunchConfiguration("init_y"),
                        "simulator.init_state_z": LaunchConfiguration("init_z"),
                    }
                ],
                remappings=[
                    ("~/odom", "/visual_slam/odom"),
                    ("~/cmd", "so3_cmd"),
                    ("~/force_disturbance", "force_disturbance"),
                    ("~/moment_disturbance", "moment_disturbance"),
                ],
            ),
            Node(
                package="so3_control",
                executable="so3_control_node",
                name="so3_control",
                output="screen",
                parameters=[
                    {
                        "mass": 0.98,
                        "use_external_yaw": False,
                        "gains.rot.x": 1.0,
                        "gains.rot.y": 1.0,
                        "gains.rot.z": 1.0,
                        "gains.ang.x": 0.07,
                        "gains.ang.y": 0.07,
                        "gains.ang.z": 0.1,
                        "corrections.z": 0.0,
                        "corrections.r": 0.0,
                        "corrections.p": 0.0,
                    }
                ],
                remappings=[
                    ("~/odom", "/state_ukf/odom"),
                    ("~/position_cmd", "/planning/pos_cmd"),
                    ("~/motors", "motors"),
                    ("~/corrections", "corrections"),
                    ("~/so3_cmd", "so3_cmd"),
                ],
            ),
            Node(
                package="so3_disturbance_generator",
                executable="so3_disturbance_generator",
                name="so3_disturbance_generator",
                output="screen",
                remappings=[
                    ("~/odom", "/visual_slam/odom"),
                    ("~/noisy_odom", "/state_ukf/odom"),
                    ("~/correction", "/visual_slam/correction"),
                    ("~/force_disturbance", "force_disturbance"),
                    ("~/moment_disturbance", "moment_disturbance"),
                ],
            ),
            Node(
                package="odom_visualization",
                executable="odom_visualization",
                name="odom_visualization",
                output="screen",
                parameters=[
                    {
                        "color.a": 1.0,
                        "color.r": 0.0,
                        "color.g": 0.0,
                        "color.b": 1.0,
                        "covariance_scale": 100.0,
                        "mesh_resource": "package://odom_visualization/meshes/hummingbird_rviz2.mesh",
                        "robot_scale": 1.0,
                        "publish_simple_model": False,
                    }
                ],
                remappings=[("~/odom", "/visual_slam/odom")],
            ),
            Node(
                package="local_sensing_node",
                executable="pcl_render_node",
                name="pcl_render_node",
                output="screen",
                parameters=[
                    {
                        "cam_width": 640,
                        "cam_height": 480,
                        "cam_fx": 387.229,
                        "cam_fy": 387.229,
                        "cam_cx": 321.046,
                        "cam_cy": 243.449,
                        "sensing_horizon": 5.0,
                        "sensing_rate": 15.0,
                        "estimation_rate": 15.0,
                        "map.x_size": LaunchConfiguration("map_size_x_"),
                        "map.y_size": LaunchConfiguration("map_size_y_"),
                        "map.z_size": LaunchConfiguration("map_size_z_"),
                    }
                ],
                remappings=[
                    ("~/global_map", "/map_generator/global_cloud"),
                    ("~/odometry", LaunchConfiguration("odometry_topic")),
                ],
            ),
        ]
    )
