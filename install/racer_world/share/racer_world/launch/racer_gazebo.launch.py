import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    world = DeclareLaunchArgument(
        'world',
        default_value='racetrack_decorated',
        description='Gazebo world file name (e.g., racetrack_decorated.world)'
    )
    paused = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused state'
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo UI'
    )
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Launch Gazebo in headless mode'
    )
    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Launch Gazebo in debug mode'
    )
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Launch Gazebo in verbose mode'
    )
    use_gpu = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU for Gazebo rendering'
    )
    visualize_lidar = DeclareLaunchArgument(
        'visualize_lidar',
        default_value='false',
        description='Visualize lidar in RViz'
    )
    laser_sample_count = DeclareLaunchArgument(
        'laser_sample_count',
        default_value='1080',
        description='Number of laser samples'
    )
    extra_gazebo_args = DeclareLaunchArgument(
        'extra_gazebo_args',
        default_value='',
        description='Extra arguments for Gazebo'
    )

    # Get package directories
    racer_description_path = get_package_share_directory('racer_description')
    racer_world_path = get_package_share_directory('racer_world')
    # Assuming gazebo_ros is the ROS 2 equivalent for gazebo_ros_pkgs
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    racer_control_path = get_package_share_directory('racer_control')

    # Xacro command for robot_description
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            os.path.join(racer_description_path, 'urdf', 'racer.xacro'),
            ' use_gpu:=', LaunchConfiguration('use_gpu'),
            ' visualize_lidar:=', LaunchConfiguration('visualize_lidar'),
            ' laser_sample_count:=', LaunchConfiguration('laser_sample_count'),
            ' simulation:=true'
        ]),
        value_type=str
    )

    # Include empty_world.launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py') # Assuming gazebo.launch.py exists
        ),
        launch_arguments={
            'world': os.path.join(racer_world_path, 'worlds', LaunchConfiguration('world') + '.world'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'debug': LaunchConfiguration('debug'),
            'verbose': LaunchConfiguration('verbose'),
            'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args'),
        }.items()
    )

    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'racer'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Include racer_control.launch (will need to be converted to .launch.py)
    racer_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(racer_control_path, 'launch', 'racer_control.launch.py') # Assuming it will be converted
        )
    )

    return LaunchDescription([
        world,
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        verbose,
        use_gpu,
        visualize_lidar,
        laser_sample_count,
        extra_gazebo_args,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        racer_control_launch,
    ])
