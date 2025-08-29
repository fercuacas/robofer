# launch/robofer.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="robofer", executable="eyes_unified_node", name="eyes"),
        Node(package="robofer", executable="state_handler_node", name="state_handler"),
        Node(package="robofer", executable="buttons_node", name="buttons"),
        Node(package="robofer", executable="wifi_manager_node", name="wifi_manager"),
        Node(package="robofer", executable="bluetooth_manager_node", name="bluetooth_manager"),
        Node(package="robofer", executable="bt_provision_node", name="bt_provision")

    ])

