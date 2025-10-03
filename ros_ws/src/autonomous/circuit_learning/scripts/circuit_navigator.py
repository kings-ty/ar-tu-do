#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from drive_msgs.msg import drive_param
import json
import os
import math
import numpy as np

class CircuitNavigator(Node):
    def __init__(self):
        super().__init__('circuit_navigator')
        
        # 구독자 설정
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # 목표 설정 구독자 (RViz 2D Nav Goal)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 드라이브 명령 퍼블리셔
        self.drive_publisher = self.create_publisher(
            drive_param,
            '/input/drive_param/autonomous',
            10
        )
        
        # 상태 변수
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.navigating = False
        self.goal_tolerance = 0.5  # 목표 도달 허용 거리
        
        # 제어 파라미터
        self.max_velocity = 0.3
        self.min_velocity = 0.1
        self.max_angle = 1.0
        
        # 웨이포인트 파일 로드
        self.load_latest_waypoints()
        
        # 타이머 설정
        self.timer = self.create_timer(0.1, self.navigation_control)
        
        self.get_logger().info('Circuit Navigator 노드가 시작되었습니다.')
        self.get_logger().info('RViz에서 2D Nav Goal을 설정하면 자율주행을 시작합니다.')
        
    def load_latest_waypoints(self):
        """가장 최근의 웨이포인트 파일을 로드"""
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        data_dir = os.path.join(package_dir, 'data')
        
        if not os.path.exists(data_dir):
            self.get_logger().warn('데이터 디렉토리가 없습니다. 먼저 웨이포인트를 기록해주세요.')
            return
            
        waypoint_files = [f for f in os.listdir(data_dir) if f.startswith('waypoints_')]
        
        if not waypoint_files:
            self.get_logger().warn('웨이포인트 파일이 없습니다. 먼저 웨이포인트를 기록해주세요.')
            return
            
        # 가장 최근 파일 선택
        latest_file = sorted(waypoint_files)[-1]
        waypoint_path = os.path.join(data_dir, latest_file)
        
        try:
            with open(waypoint_path, 'r') as f:
                self.waypoints = json.load(f)
            self.get_logger().info(f'웨이포인트 로드 완료: {len(self.waypoints)}개 ({latest_file})')
        except Exception as e:
            self.get_logger().error(f'웨이포인트 로드 실패: {e}')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def goal_callback(self, msg):
        """목표 설정시 자율주행 시작"""
        if not self.waypoints:
            self.get_logger().warn('웨이포인트가 없습니다. 먼저 기록해주세요.')
            return
            
        if not self.navigating:
            self.start_navigation()
        else:
            self.stop_navigation()
    
    def start_navigation(self):
        """자율주행 시작"""
        self.navigating = True
        self.current_waypoint_index = 0
        self.get_logger().info('🚗 자율주행 시작! 서킷을 따라 주행합니다.')
        
    def stop_navigation(self):
        """자율주행 정지"""
        self.navigating = False
        
        # 정지 명령 전송
        stop_msg = drive_param()
        stop_msg.angle = 0.0
        stop_msg.velocity = 0.0
        self.drive_publisher.publish(stop_msg)
        
        self.get_logger().info('⏹️  자율주행 정지.')
    
    def navigation_control(self):
        """자율주행 제어 로직"""
        if not self.navigating or not self.waypoints or self.current_pose is None:
            return
            
        # 현재 위치
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        # 현재 목표 웨이포인트
        if self.current_waypoint_index >= len(self.waypoints):
            # 서킷 완주
            self.get_logger().info('🏁 서킷 완주! 자율주행을 정지합니다.')
            self.stop_navigation()
            return
            
        target_waypoint = self.waypoints[self.current_waypoint_index]
        target_x = target_waypoint['pose']['x']
        target_y = target_waypoint['pose']['y']
        
        # 목표까지의 거리 계산
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 목표에 도달했는지 확인
        if distance < self.goal_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f'웨이포인트 {self.current_waypoint_index}/{len(self.waypoints)} 도달')
            return
            
        # 조향각 계산
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        
        # 현재 차량의 방향 (쿼터니언에서 yaw 추출)
        q = self.current_pose.pose.pose.orientation
        current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # 조향각 오차 계산
        angle_error = angle_to_target - current_yaw
        
        # 각도를 -π ~ π 범위로 정규화
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        # 제어 명령 계산
        steering_angle = np.clip(angle_error * 2.0, -self.max_angle, self.max_angle)
        
        # 속도 계산 (조향각이 클수록 느리게)
        velocity = self.max_velocity * (1.0 - abs(steering_angle) / self.max_angle * 0.5)
        velocity = max(velocity, self.min_velocity)
        
        # 명령 전송
        drive_msg = drive_param()
        drive_msg.angle = steering_angle
        drive_msg.velocity = velocity
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = CircuitNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('키보드 인터럽트로 종료.')
        if navigator.navigating:
            navigator.stop_navigation()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()