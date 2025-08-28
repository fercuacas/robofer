from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robofer',
            executable='eyes_unified_node',
            name='eyes_unified_node',
            output='screen',
            parameters=[{'backend': 'sim'}],
        ),
        Node(
            package='robofer',
            executable='state_handler_node',
            name='state_handler',
            output='screen',
            parameters=[{'sim': True}],
        ),
        Node(
            package='robofer',
            executable='wifi_manager_node',
            name='wifi_manager',
            output='screen',
        ),
    ])
