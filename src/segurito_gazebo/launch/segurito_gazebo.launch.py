from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
os.environ["GAZEBO_PLUGIN_PATH"] = ":".join(
    p for p in os.environ.get("GAZEBO_PLUGIN_PATH", "").split(":")
    if "force_system" not in p)

def generate_launch_description():
    pkg_gz   = get_package_share_directory('gazebo_ros')
    pkg_desc = get_package_share_directory('segurito_description')

    # Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(
            get_package_share_directory('segurito_gazebo'),
            'worlds', 'empty.world'), 'verbose': 'true'}.items())

    # Publicar robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ',
              PathJoinSubstitution([pkg_desc, 'urdf', 'segurito.urdf.xacro'])])
        }],
        output='screen')

    # Spawner: inserta el modelo en Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'segurito'],
        output='screen')

    # Spawner controlador diff-drive
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file',
            PathJoinSubstitution([
                FindPackageShare('segurito_description'),
                'config', 'diff_drive.yaml'
            ])
        ],
        output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity,
                          on_exit=[controller_spawner])),
    ])
