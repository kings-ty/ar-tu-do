from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables specific to simulation
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack',
        description='Name of the world to launch'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Include the Gazebo launch file from racer_description
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('racer_description'),
                'launch',
                'spawn_racer_in_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('racer_slam'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(slam_launch)

    return ld