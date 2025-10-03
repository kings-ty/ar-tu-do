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

    # XACRO 파일 경로 설정
    urdf_path = os.path.join(pkg_racer_description, 'urdf', 'racer.xacro')

    # Launch 구성 변수 선언
    # Gazebo와 함께 사용하려면 use_sim_time을 'true'로 설정해야 합니다.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot State Publisher 설정 (기존 코드와 동일)
    # XACRO 파일을 처리하여 robot_description 파라미터로 로드합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path, ' simulation:=true']),
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo 서버 및 클라이언트 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # Gazebo에 로봇 모델 스폰(spawn)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'racer_robot'],
        output='screen'
    )

    return LaunchDescription([
        # use_sim_time 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # 노드 및 런치 파일 실행 순서 정의
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
    ])