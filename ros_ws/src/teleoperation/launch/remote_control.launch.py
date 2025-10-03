#!/usr/bin/env python3

"""
ROS 2 Launch file for teleoperation package.
Launches all necessary nodes that let the user control the car with the keyboard and/or gamepad.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    joystick_type_arg = DeclareLaunchArgument(
        'joystick_type',
        default_value='xbox360',
        description='Joystick type: ps3, xbox360, xboxone, or custom'
    )
    
    joystick_steering_axis_arg = DeclareLaunchArgument(
        'joystick_steering_axis',
        default_value='0',
        description='Joystick steering axis (used when joystick_type is custom)'
    )
    
    joystick_acceleration_axis_arg = DeclareLaunchArgument(
        'joystick_acceleration_axis', 
        default_value='13',
        description='Joystick acceleration axis (used when joystick_type is custom)'
    )
    
    joystick_deceleration_axis_arg = DeclareLaunchArgument(
        'joystick_deceleration_axis',
        default_value='12', 
        description='Joystick deceleration axis (used when joystick_type is custom)'
    )
    
    joystick_enable_manual_button_arg = DeclareLaunchArgument(
        'joystick_enable_manual_button',
        default_value='14',
        description='Joystick manual mode enable button (used when joystick_type is custom)'
    )
    
    joystick_enable_autonomous_button_arg = DeclareLaunchArgument(
        'joystick_enable_autonomous_button',
        default_value='13',
        description='Joystick autonomous mode enable button (used when joystick_type is custom)'
    )


    # Keyboard controller node
    keyboard_controller_node = Node(
        package='teleoperation',
        executable='keyboard_controller',
        name='keyboard_controller',
        output='log'
    )

    # Converter node
    converter_node = ExecuteProcess(
        cmd=['python3', '/home/ty/f1test_ws/ar-tu-do/drive_param_to_ackermann.py'],
        name='drive_param_to_ackermann_converter',
        output='screen'
    )

    return LaunchDescription([
        keyboard_controller_node,
        converter_node,
    ])