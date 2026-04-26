from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([FindPackageShare("plan_manage"), "config", "traj.rviz"])

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rvizvisualisation",
                output="log",
                arguments=["-d", rviz_config],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_53",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "navigation",
                ],
            ),
        ]
    )
