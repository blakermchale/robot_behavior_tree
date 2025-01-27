#!/usr/bin/env python3
'''
runner.launch.py

Creates lifecycle node for running behavior trees.
'''


from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

import os

from ros2_utils.launch_manager import LaunchManager


# Get relative package directories
robot_behavior_tree = get_package_share_directory("robot_behavior_tree")


DEFAULT_BT = os.path.join(robot_behavior_tree, "trees", "test_actions.xml")
DEFAULT_CONFIG = os.path.join(robot_behavior_tree, "config", "bt_runner.yaml")


def launch_setup(args):
    """Allows declaration of launch arguments within the ROS2 context
    """
    lm = LaunchManager()
    # Change to trees folder for including subtrees since behavior trees xml parsing uses cwd
    os.chdir(os.path.dirname(DEFAULT_BT))

    namespace = args.namespace
    lm.add_action_list([
        # Node(
        #     package='robot_behavior_tree', executable="bt_runner",
        #     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
        #     output='screen',
        #     namespace=namespace,
        #     parameters=[{'use_sim_time': True},
        #                 {'default_server_timeout': 100},
        #                 {'error_code_names': ['']}]),
        Node(
            package='robot_behavior_tree', executable="bt_runner", name="bt_runner",
            # condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            output='screen',
            namespace=namespace,
            parameters=[LaunchConfiguration("config_file")]),
    ])
    return lm.describe_sub_entities()


def combine_names(l: list, sep: str):
    l = list(filter(None, l))  # Remove empty strings
    return sep.join(l)


def generate_launch_description():
    lm = LaunchManager()
    lm.add_arg("config_file", DEFAULT_CONFIG, "Path to yaml file that contains namespaces for a vehicle and their relevant launch args. See examples/ folder.")
    lm.add_arg("default_run_bt_xml", DEFAULT_BT, "Behavior tree xml file to load initially.")
    lm.add_arg("enable_groot_monitoring", True, "Enable Groot monitoring.")
    lm.add_arg("groot_zmq_publisher_port", 1666, "Groot publisher port.")
    lm.add_arg("groot_zmq_server_port", 1667, "Groot publisher port.")
    lm.add_arg("namespace", "", "Namespace for ROS.")
    lm.add_opaque_function(launch_setup)
    return lm
