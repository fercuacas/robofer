# launch/robofer.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robofer', executable='eyes_node',   name='eyes'),
        Node(package='robofer', executable='servos_node', name='servos'),
        Node(package='robofer', executable='sound_node',  name='sound'),
        Node(package='robofer', executable='buttons_node',name='buttons'),
    ])