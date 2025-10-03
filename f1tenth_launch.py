#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robot.urdf 파일의 절대 경로를 직접 지정
    urdf_path = '/home/ty/f1test_ws/ar-tu-do/robot.urdf'
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo 실행
    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
             'world:=/home/ty/f1test_ws/ar-tu-do/ros_ws/install/racer_world/share/racer_world/worlds/racetrack_decorated.world'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 차량 스폰
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'racer', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1']
    )

    # Joint State Broadcaster 스포너
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Controller Spawners
    spawn_left_steering_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_steering_controller'],
        output='screen'
    )
    spawn_right_steering_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_steering_controller'],
        output='screen'
    )
    spawn_left_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_wheel_controller'],
        output='screen'
    )
    spawn_right_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_wheel_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_joint_state_broadcaster,
        spawn_left_steering_controller,
        spawn_right_steering_controller,
        spawn_left_wheel_controller,
        spawn_right_wheel_controller,
    ])