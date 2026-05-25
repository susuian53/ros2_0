import os
from time import sleep
import launch
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import xacro
import math


def launch_setup(context, *args, **kwargs):
    # pkg
    pkg_share = get_package_share_directory('cyberdog_gazebo')
    pkg_prefix = get_package_prefix('cyberdog_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # config
    hang_robot = LaunchConfiguration('hang_robot').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    wname = LaunchConfiguration('wname').perform(context)
    rname = LaunchConfiguration('rname').perform(context)
    initial_x = LaunchConfiguration('initial_x').perform(context)
    initial_y = LaunchConfiguration('initial_y').perform(context)
    initial_yaw = LaunchConfiguration('initial_yaw').perform(context)

    # env
    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, 'model')
    my_env["GAZEBO_PLUGIN_PATH"] = os.path.join(get_package_prefix('cyberdog_gazebo'), 'lib')

    # world
    world_path = os.path.join(pkg_share, 'world', wname+'.world')

    # urdf
    xacro_path = os.path.join(get_package_share_directory(
        rname+'_description'), 'xacro', 'robot.xacro')
    urdf_contents = xacro.process_file(xacro_path, mappings={
                                       'DEBUG': hang_robot, 'USE_LIDAR': use_lidar, 'USE_CAMERA': use_camera}).toprettyxml(indent='  ')

    # spawn
    half_yaw = float(initial_yaw) / 2.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    spawn_entity_message_contents = "'{initial_pose:{ position: {x: " + initial_x + ", y: " + initial_y + ", z: 0.6}, orientation: {x: 0.0, y: 0.0, z: " + str(qz) + ", w: " + str(qw) + "}},  name: \"robot\", xml: \"" + \
        urdf_contents.replace('"', '\\"') + "\"}'"
    spawn_entity = launch.actions.ExecuteProcess(
        name='spawn_entity', cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_message_contents], env=my_env, shell=True, log_cmd=False)

    # gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_simulator')),
        launch_arguments={
            'world': world_path,
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'env': my_env
        }.items()
    )

    # gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(
            [LaunchConfiguration('use_simulator'), ' and not ', LaunchConfiguration('headless')]))
    )

    return [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity]


def generate_launch_description():
    ld = launch.LaunchDescription([
        DeclareLaunchArgument(
            name='headless',
            default_value='False',
            description='Whether to execute gzclient'
        ),
        DeclareLaunchArgument(
            name='use_simulator',
            default_value='True',
            description='Whether to start the simulator'
        ),
        DeclareLaunchArgument(
            name='paused',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='hang_robot',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='use_lidar',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='use_camera',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='rname',
            default_value='cyberdog'
        ),
        DeclareLaunchArgument(
            name='wname',
            default_value='race'
        ),
        DeclareLaunchArgument(
            name='initial_x',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='initial_y',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='initial_yaw',
            default_value='0.0'
        ),
        OpaqueFunction(function=launch_setup)
    ])
    return ld
