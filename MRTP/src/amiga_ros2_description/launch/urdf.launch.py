import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def qualify_ros(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)

    return [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=ns,
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_joints")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command(
                            [
                                "xacro ",
                                LaunchConfiguration("urdf"),
                                " use_lidar:=",
                                LaunchConfiguration("use_lidar"),
                                " gps_link_name:=",
                                LaunchConfiguration("gps_link_name"),
                                " use_vectornav:=",
                                LaunchConfiguration("use_vectornav"),
                            ]
                        ),
                        value_type=str,
                    )
                }
            ],
            # tf2_ros hardcodes /tf, /tf_static as absolute regardless of
            # node namespace — without this remap, robot_state_publisher
            # keeps broadcasting the URDF's static transforms to the global
            # root /tf_static for every robot, so a namespaced robot's own
            # /<ns>/tf_static stays empty and anything needing its static
            # frames (e.g. EKF resolving an IMU frame into base_link) never
            # gets them.
            remappings=[
                ("joint_states", LaunchConfiguration("joint_states_topic")),
                ("/tf", qualify_ros(ns, "tf")),
                ("/tf_static", qualify_ros(ns, "tf_static")),
            ],
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("amiga_ros2_description")
    urdf_path = os.path.join(package_dir, "urdf", "amiga_descr.urdf.xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="urdf", default_value=urdf_path, description="Path to robot URDF"
            ),
            DeclareLaunchArgument(
                name="publish_joints",
                default_value="true",
                description="Publish joint states",
            ),
            DeclareLaunchArgument(
                name="use_lidar",
                default_value="false",
                description="Whether to include the LiDAR in the robot description",
            ),
            DeclareLaunchArgument(
                name="gps_link_name",
                default_value="gps_link",
                description="Name of the GPS link (gps_link or gps_antenna)",
            ),
            DeclareLaunchArgument(
                name="use_vectornav",
                default_value="false",
                description="Whether to use VectorNav IMU instead of BNO085",
            ),
            DeclareLaunchArgument(
                name="joint_states_topic",
                default_value="/joint_states",
                description="Joint-state topic for robot_state_publisher",
            ),
            DeclareLaunchArgument(
                name="namespace",
                default_value="",
                description="ROS namespace for the RSP/JSP nodes (per-robot, e.g. 'amiga1')",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
