from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_names = [
        ("motion_sensor_node", []),
        ("camera_vision_node", []),
        ("battery_monitor_node", []),
        ("encoder_node", []),
        ("odometry_node", []),
        ("slam_node", []),
        ("map_server_node", []),
        ("navigation_node", []),
        ("motor_controller_node", []),
        ("alert_manager_node", []),
    ]
    return LaunchDescription([
        Node(package='segurito', executable=name, name=name, parameters=params)
        for name, params in node_names
    ])