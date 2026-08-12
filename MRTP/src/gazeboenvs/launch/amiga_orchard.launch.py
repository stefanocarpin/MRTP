# adapted by Stefano Carpin for https://github/stefanocarpin/MRTP
# from amiga-ros2-bridge/amiga_ros2_gazebo/launch/gazebo.launch.py
# (single-robot subset, retargeted from Ignition Fortress (ign gazebo /
# ign_ros2_control) to Gazebo Harmonic (gz sim / gz_ros2_control) to match
# the ROS 2 Jazzy + Gazebo toolchain already used elsewhere in gazeboenvs.
#
# Brings up the Amiga + Kinova Gen3 simulator in the orchard_nbv world:
#   * gz sim with the orchard world (144 always-loaded citrus trees)
#   * one Amiga+Kinova robot, spawned from models/amiga_kinova/model.sdf
#   * the diff-drive base and Kinova arm/gripper ros2_control controllers
#   * a ros_gz_bridge exposing clock, IMU, GPS, camera and lidar topics
#
# Unlike the upstream launch file this does not bring up localization,
# Nav2, MoveIt or the behavior-tree mission layer -- those live in
# amiga-ros2-bridge packages that are not part of this workspace. This
# file only owns the simulation environment, matching the scope of every
# other launch file in gazeboenvs.
#
# Requires amiga_ros2_description (vendored in this workspace under
# MRTP/src/amiga_ros2_description), plus kortex_description and
# robotiq_description built and sourced from ws_moveit: the robot model's
# meshes and the ros2_control joint description are resolved from those
# packages at launch time via package:// URIs and xacro, exactly as in
# the upstream package.

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('headless', default_value='false',
                          choices=['true', 'false'],
                          description='Run gz sim server-only, no GUI.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='amiga_kinova',
                          description='Name the robot is spawned under.'),
]

for pose_element, default in [('x', '-5.0'), ('y', '-3.0'), ('yaw', '0.0')]:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value=default,
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.05',
                 description='z component of the robot pose.'))


def resolve_package_uris(content: str) -> str:
    """Rewrite package:// URIs to file:// -- gz sim cannot resolve them."""

    def replacer(match):
        pkg, rest = match.group(1), match.group(2)
        try:
            return f"file://{get_package_share_directory(pkg)}/{rest}"
        except PackageNotFoundError:
            print(f"[gazeboenvs] WARNING: package not found: {pkg}")
            return match.group(0)

    return re.sub(r"package://([^/]+)/(.+?)(?=[<\"\s])", replacer, content)


def launch_setup(context, *args, **kwargs):
    pkg_gazeboenvs = get_package_share_directory('gazeboenvs')

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = {'use_sim_time': True}

    # ── Resolve world SDF (package:// -> file://) ────────────────────────
    world_path = os.path.join(pkg_gazeboenvs, 'worlds', 'orchard_nbv.sdf')
    with open(world_path) as f:
        world_content = f.read()
    tmp_world = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix='amiga_orchard_resolved_', delete=False)
    tmp_world.write(resolve_package_uris(world_content))
    tmp_world.flush()

    # ── Resolve robot model SDF ────────────────────────────────────────
    model_path = os.path.join(pkg_gazeboenvs, 'models', 'amiga_kinova', 'model.sdf')
    controllers_yaml = os.path.join(pkg_gazeboenvs, 'config', 'ros2_controllers_sim.yaml')
    with open(model_path) as f:
        model_content = f.read()
    model_content = model_content.replace(
        '$(find-pkg-share gazeboenvs)/config/ros2_controllers_sim.yaml',
        controllers_yaml)
    model_content = resolve_package_uris(model_content)
    tmp_model = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix=f'{robot_name}_resolved_', delete=False)
    tmp_model.write(model_content)
    tmp_model.flush()

    # ── Combined sim URDF for gz_ros2_control (TF/description muted) ────
    # Only feeds gz_ros2_control's joint/hardware-interface lookup (see
    # <robot_param_node>/<robot_param> in model.sdf), never used for TF.
    pkg_amiga_descr = get_package_share_directory('amiga_ros2_description')
    xacro_path = os.path.join(pkg_amiga_descr, 'urdf', 'sim', 'amiga_kinova_sim.urdf.xacro')
    result = subprocess.run(
        ['xacro', xacro_path, 'sim_ignition:=true'], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f'xacro failed for {xacro_path}:\n{result.stderr}')
    sim_robot_description = result.stdout

    # ── Gazebo ────────────────────────────────────────────────────────
    gz_cmd = ['gz', 'sim', '-r']
    if headless:
        gz_cmd += ['-s', '--headless-rendering']
    gz_cmd.append(tmp_world.name)
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen')

    # robot_state_publisher instance that only exists to hand gz_ros2_control
    # the sim URDF's <ros2_control> block (see robot_param_node/robot_param
    # in model.sdf). Its /tf and /tf_static are kept off the default names so
    # they never collide with anything that expects the real robot TF, but
    # robot_description is left unremapped: model.sdf's gz_ros2_control
    # plugin (<robot_param_node>gz_description_server</robot_param_node>,
    # <robot_param>robot_description</robot_param>) subscribes to the plain
    # 'robot_description' topic name regardless of node namespace, so a
    # remap here would leave controller_manager waiting forever for data
    # that never arrives on the topic it's actually subscribed to.
    gz_description_server = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gz_description_server',
        output='screen',
        parameters=[use_sim_time, {'robot_description': sim_robot_description}],
        remappings=[
            ('/tf', 'gz_sim/tf'),
            ('/tf_static', 'gz_sim/tf_static'),
            ('joint_states', 'gz_sim/joint_states'),
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', tmp_model.name,
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')],
        output='screen',
        parameters=[use_sim_time],
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager-timeout', '120'],
            output='screen',
            parameters=[use_sim_time],
        )

    spawn_jsb = spawner('joint_state_broadcaster')
    spawn_diff = spawner('diff_drive_controller')
    spawn_jtc = spawner('joint_trajectory_controller')
    spawn_gripper = spawner('robotiq_gripper_controller')

    # ── gz <-> ROS bridge (raw sim topics, gz-side truth) ────────────────
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/chassis/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # oak0 (front)
        '/oak_camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/oak_camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/oak_camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/oak_camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        # oak1 (back)
        '/oak_camera_back/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/oak_camera_back/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/oak_camera_back/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/oak_camera_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        # Kinova wrist camera
        '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        # 3D lidar
        '/ouster/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
        output='screen',
        parameters=[use_sim_time],
        arguments=bridge_args,
    )

    return [
        gazebo,
        gz_description_server,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_diff, on_exit=[spawn_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jtc, on_exit=[spawn_gripper])),
        bridge,
    ]


def generate_launch_description():
    pkg_gazeboenvs = get_package_share_directory('gazeboenvs')

    # gz-transport discovery uses UDP multicast on whatever interface GZ_IP
    # points to. If the environment (e.g. ~/.bashrc) sets GZ_IP to a LAN
    # address, multicast getting filtered by the network breaks discovery
    # between gz sim and ros_gz_sim/ros_gz_bridge even though they're on the
    # same host -- this hangs "create" forever on "Requesting list of world
    # names.". Force loopback here so the sim doesn't depend on shell state.
    set_env_vars_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_gazeboenvs, 'models'))

    # Belt-and-suspenders: gz_ros2_control's plugin normally resolves via
    # LD_LIBRARY_PATH once the workspace is sourced, but make sure gz sim's
    # own plugin search path also covers it.
    try:
        gz_ros2_control_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
        set_env_vars_plugins = AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', gz_ros2_control_lib)
    except PackageNotFoundError:
        set_env_vars_plugins = None

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(set_env_vars_gz_ip)
    ld.add_action(set_env_vars_resources)
    if set_env_vars_plugins is not None:
        ld.add_action(set_env_vars_plugins)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
