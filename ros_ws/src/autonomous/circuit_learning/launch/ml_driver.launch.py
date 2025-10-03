from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ML 자율주행 드라이버
        Node(
            package='circuit_learning',
            executable='ml_driver.py',
            name='ml_driver',
            output='screen'
        )
    ])