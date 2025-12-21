# launch/robofer.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    eyes_node = Node(
        package="robofer",
        executable="eyes_unified_node",
        name="eyes",
        output="screen",
    )

    state_handler_node = Node(
        package="robofer",
        executable="state_handler_node",
        name="state_handler",
        output="screen",
        parameters=[{
            # Hardware config para Orange Pi + servo en pin físico 7 (GPIO S/N 73)
            "sim": False,
            "gpiochip": "gpiochip0",
            "servo1_offset": 73,
            "servo2_offset": 74,
            "servo1_neutral_deg": 75.0,
            "servo2_neutral_deg": 80.0,
        }],
    )

    buttons_node = Node(
        package="robofer",
        executable="buttons_node",
        name="buttons",
        output="screen",
        # Usa los defaults del nodo (gpiochip0 y offsets 71/70/69/72),
        # que ya son los que te funcionan en la Orange Pi.
    )

    wifi_node = Node(
        package="robofer",
        executable="wifi_manager_node",
        name="wifi_manager",
        output="screen",
    )

    bt_manager_node = Node(
        package="robofer",
        executable="bluetooth_manager_node",
        name="bluetooth_manager",
        output="screen",
    )

    bt_provision_node = Node(
        package="robofer",
        executable="bt_provision_node",
        name="bt_provision",
        output="screen",
    )

    return LaunchDescription([
        eyes_node,
        state_handler_node,
        buttons_node,
        wifi_node,
        bt_manager_node,
        bt_provision_node,
    ])


