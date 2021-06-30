#!/usr/bin/env python3
'''
runner.launch.py

Creates lifecycle node for running behavior trees.
'''


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

import os


# Get relative package directories
robot_behavior_tree = get_package_share_directory("robot_behavior_tree")


def generate_launch_description():   
    launch_description = []
    launch_description += get_launch_arguments()
    launch_description += [OpaqueFunction(function = launch_setup)] 
    return LaunchDescription(launch_description)


DEFAULT_BT = os.path.join(robot_behavior_tree, "trees", "test_actions.xml")
# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "config_file",               "default": "",          "description": "Config file bt runner parameters"},
    {"name": "default_bt_xml_filename",   "default": DEFAULT_BT,  "description": "Behavior tree xml file to load initially"},
    {"name": "enable_groot_monitoring",   "default": "true",      "description": "Enable groot monitoring."},
    {"name": "groot_zmq_publisher_port",  "default": "1666",      "description": "Groot publisher port."},
    {"name": "groot_zmq_server_port",     "default": "1667",      "description": "Groot publisher port."},
]
def get_launch_arguments():
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description'], choices=param.get("choices")) for param in LAUNCH_ARGS]


def set_configurable_parameters():
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in LAUNCH_ARGS])


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    namespace = "drone_0"  # TODO: use arg for namespace
    lifecycle_nodes = [f"/{namespace}/bt_runner"]
    ld = [
        Node(
            package='robot_behavior_tree', executable="bt_runner",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
            output='screen',
            namespace=namespace,
            parameters=[set_configurable_parameters()]),
        Node(
            package='robot_behavior_tree', executable="bt_runner", name="bt_runner",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            output='screen',
            namespace=namespace,
            parameters=[set_configurable_parameters(),
                        LaunchConfiguration("config_file")]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ]
    return ld
