from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_left_steering',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'left_steering', '100'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_right_steering',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'right_steering', '100'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_left_wheel_back',
            arguments=['-0.165', '0.09', '0.05', '0', '0', '0', '1', 'base_link', 'left_wheel_back', '100'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_left_wheel_front',
            arguments=['0.165', '0.09', '0.05', '0', '0', '0', '1', 'base_link', 'left_wheel_front', '100'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_right_wheel_back',
            arguments=['-0.165', '-0.09', '0.05', '0', '0', '0', '1', 'base_link', 'right_wheel_back', '100'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_right_wheel_front',
            arguments=['0.165', '-0.09', '0.05', '0', '0', '0', '1', 'base_link', 'right_wheel_front', '100'],
        ),
    ])