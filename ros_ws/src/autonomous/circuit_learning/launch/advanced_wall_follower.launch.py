import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 설정
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_racer_description = get_package_share_directory('racer_description')
    pkg_racer_slam = get_package_share_directory('racer_slam')

    # XACRO 파일 경로 설정
    urdf_path = os.path.join(pkg_racer_description, 'urdf', 'racer.xacro')
    
    # World 파일 경로 설정
    world_path = '/home/ty/f1test_ws/ar-tu-do/install/racer_world/share/racer_world/worlds/racetrack_decorated.world'

    # Launch 구성 변수 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_racer_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path, ' simulation:=true']),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo에 로봇 모델 스폰
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'racer_robot'],
        output='screen'
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_racer_slam, 'config', 'slam_rviz.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 고급 Wall Follower
    advanced_wall_follower_node = Node(
        package='circuit_learning',
        executable='advanced_wall_follower.py',
        name='advanced_wall_follower',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        
        # 실행 순서
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        slam_node,
        rviz_node,
        advanced_wall_follower_node,
    ])