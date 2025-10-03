import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='racetrack_decorated_2',
        description='Gazebo world file name (e.g., racetrack_decorated_2.world)'
    )
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused state'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo UI'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Launch Gazebo in debug mode'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Launch Gazebo in verbose mode'
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU for Gazebo rendering'
    )
    joystick_type_arg = DeclareLaunchArgument(
        'joystick_type',
        default_value='xbox360',
        description='Type of joystick to use for teleoperation'
    )
    visualize_lidar_arg = DeclareLaunchArgument(
        'visualize_lidar',
        default_value='false',
        description='Visualize lidar in RViz'
    )
    emergency_stop_arg = DeclareLaunchArgument(
        'emergency_stop',
        default_value='true',
        description='Enable emergency stop functionality'
    )
    mode_override_arg = DeclareLaunchArgument(
        'mode_override',
        default_value='0',
        description='Mode Override: 0=user input, 1=manual, 2=autonomous'
    )

    # Get package share directories
    racer_world_path = get_package_share_directory('racer_world')
    simulation_tools_path = get_package_share_directory('simulation_tools')
    wallfollowing2_path = get_package_share_directory('wallfollowing2')
    emergency_stop_path = get_package_share_directory('emergency_stop')
    vesc_sim_path = get_package_share_directory('vesc_sim')
    car_control_path = get_package_share_directory('car_control')
    teleoperation_path = get_package_share_directory('teleoperation')

    # Include racer_gazebo.launch.py
    racer_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(racer_world_path, 'launch', 'racer_gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'debug': LaunchConfiguration('debug'),
            'verbose': LaunchConfiguration('verbose'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'visualize_lidar': LaunchConfiguration('visualize_lidar'),
        }.items()
    )

    # RViz node
    rviz_config_file = os.path.join(racer_world_path, 'launch', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # lap_timer node
    lap_timer_node = Node(
        package='simulation_tools',
        executable='lap_timer.py',
        name='lap_timer',
        output='screen'
    )

    # Include autonomous_driving.launch (needs conversion)
    # For now, I'll assume it will be converted to .launch.py
    autonomous_driving_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wallfollowing2_path, 'launch', 'autonomous_driving.launch.py')
        )
    )

    # Emergency stop group (needs conversion)
    emergency_stop_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('emergency_stop')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(emergency_stop_path, 'launch', 'emergency_stop.launch.py')
                )
            )
        ]
    )

    # Include vesc_sim.launch (needs conversion)
    vesc_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_sim_path, 'launch', 'vesc_sim.launch.py')
        )
    )

    # Include car_control.launch (needs conversion)
    car_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_control_path, 'launch', 'car_control.launch.py')
        ),
        launch_arguments={
            'mode_override': LaunchConfiguration('mode_override'),
        }.items()
    )

    # Include remote_control.launch (needs conversion)
    remote_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleoperation_path, 'launch', 'remote_control.launch.py')
        ),
        launch_arguments={
            'joystick_type': LaunchConfiguration('joystick_type'),
        }.items()
    )

    # crash_detector node
    crash_detector_node = Node(
        package='simulation_tools',
        executable='crash_detector',
        name='crash_detector',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        debug_arg,
        verbose_arg,
        use_gpu_arg,
        joystick_type_arg,
        visualize_lidar_arg,
        emergency_stop_arg,
        mode_override_arg,
        racer_gazebo_launch,
        rviz_node,
        lap_timer_node,
        autonomous_driving_launch,
        emergency_stop_group,
        vesc_sim_launch,
        car_control_launch,
        remote_control_launch,
        crash_detector_node,
    ])
