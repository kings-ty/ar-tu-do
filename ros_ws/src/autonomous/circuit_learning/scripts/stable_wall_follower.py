#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import numpy as np
import math

class StableWallFollower(Node):
    def __init__(self):
        super().__init__('stable_wall_follower')
        
        # 구독자 설정
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # 퍼블리셔 설정
        self.drive_publisher = self.create_publisher(
            drive_param,
            '/input/drive_param/autonomous',
            10
        )
        
        # 제어 파라미터
        self.target_wall_distance = 0.8  # 벽으로부터 목표 거리 (m)
        self.max_velocity = 0.4
        self.min_velocity = 0.1
        self.max_steering = 1.0
        
        # PID 제어기 파라미터
        self.kp = 2.0  # 비례 게인
        self.ki = 0.1  # 적분 게인
        self.kd = 0.5  # 미분 게인
        
        # PID 제어기 상태
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.previous_time = self.get_clock().now()
        
        # 안전 파라미터
        self.emergency_distance = 0.3  # 긴급 정지 거리
        self.slow_down_distance = 0.6  # 속도 감소 시작 거리
        
        # 센서 구역 설정 (라디안)
        self.front_angle_range = math.pi / 6    # ±30도 (전방)
        self.left_angle_range = math.pi / 4     # 좌측 90도 범위
        self.right_angle_range = math.pi / 4    # 우측 90도 범위
        
        self.get_logger().info('안정적인 Wall Follower 시작됨')
        self.get_logger().info(f'목표 벽 거리: {self.target_wall_distance}m')
    
    def get_sensor_regions(self, ranges, angle_min, angle_max):
        """LiDAR 데이터를 구역별로 분할"""
        total_points = len(ranges)
        angle_increment = (angle_max - angle_min) / total_points
        
        # 각 구역의 인덱스 계산
        center_idx = total_points // 2
        
        # 전방 구역 (±30도)
        front_width = int(self.front_angle_range / angle_increment / 2)
        front_start = max(0, center_idx - front_width)
        front_end = min(total_points, center_idx + front_width)
        
        # 좌측 구역 (90도~180도)
        left_start = max(0, int((math.pi/2) / angle_increment))
        left_end = min(total_points, int((math.pi) / angle_increment))
        
        # 우측 구역 (0도~90도) - 인덱스는 뒤쪽
        right_start = max(0, total_points - int((math.pi/2) / angle_increment))
        right_end = total_points
        
        # 무한값 처리
        clean_ranges = np.array(ranges)
        clean_ranges[np.isinf(clean_ranges)] = 10.0  # inf를 10m로 대체
        
        # 각 구역의 평균 거리 계산
        front_distances = clean_ranges[front_start:front_end]
        left_distances = clean_ranges[left_start:left_end]
        right_distances = clean_ranges[right_start:right_end]
        
        # 최소값 사용 (가장 가까운 장애물)
        front_distance = np.min(front_distances) if len(front_distances) > 0 else 10.0
        left_distance = np.min(left_distances) if len(left_distances) > 0 else 10.0
        right_distance = np.min(right_distances) if len(right_distances) > 0 else 10.0
        
        return {
            'front': front_distance,
            'left': left_distance,
            'right': right_distance
        }
    
    def calculate_corridor_center_error(self, left_dist, right_dist):
        """복도 중앙으로부터의 오차 계산"""
        # 복도 폭 계산
        corridor_width = left_dist + right_dist
        
        # 중앙에서의 편차 계산 (음수: 오른쪽으로 치우침, 양수: 왼쪽으로 치우침)
        center_offset = (left_dist - right_dist) / 2.0
        
        return center_offset, corridor_width
    
    def calculate_wall_following_error(self, left_dist, right_dist):
        """우측 벽 따라가기 기준 오차 계산"""
        # 우측 벽을 따라가는 경우
        error = right_dist - self.target_wall_distance
        return error
    
    def calculate_steering_angle(self, error, dt):
        """PID 제어기로 조향각 계산"""
        # PID 계산
        self.integral_error += error * dt
        derivative_error = (error - self.previous_error) / dt if dt > 0 else 0
        
        # 적분 포화 방지
        self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        # PID 출력
        steering = (self.kp * error + 
                   self.ki * self.integral_error + 
                   self.kd * derivative_error)
        
        # 조향각 제한
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        
        self.previous_error = error
        return steering
    
    def calculate_adaptive_speed(self, front_dist, left_dist, right_dist, corridor_width):
        """상황에 따른 적응적 속도 계산"""
        # 기본 속도
        speed = self.max_velocity
        
        # 1. 전방 장애물에 따른 속도 조절
        if front_dist < self.emergency_distance:
            speed = 0.0  # 긴급 정지
        elif front_dist < self.slow_down_distance:
            # 선형적으로 속도 감소
            speed_factor = (front_dist - self.emergency_distance) / (self.slow_down_distance - self.emergency_distance)
            speed = self.min_velocity + (self.max_velocity - self.min_velocity) * speed_factor
        
        # 2. 복도 폭에 따른 속도 조절
        min_safe_width = 1.0  # 최소 안전 폭
        if corridor_width < min_safe_width:
            width_factor = corridor_width / min_safe_width
            speed = min(speed, self.min_velocity + (self.max_velocity - self.min_velocity) * width_factor)
        
        # 3. 좌우 불균형에 따른 속도 조절
        balance_ratio = min(left_dist, right_dist) / max(left_dist, right_dist)
        if balance_ratio < 0.3:  # 매우 불균형한 경우
            speed *= 0.7
        
        return max(speed, 0.0)
    
    def laser_callback(self, msg):
        """LiDAR 데이터 처리 및 제어 명령 생성"""
        # 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = current_time
        
        if dt <= 0:
            return
        
        # 센서 구역 분석
        regions = self.get_sensor_regions(msg.ranges, msg.angle_min, msg.angle_max)
        front_dist = regions['front']
        left_dist = regions['left']
        right_dist = regions['right']
        
        # 복도 중앙 및 폭 계산
        center_error, corridor_width = self.calculate_corridor_center_error(left_dist, right_dist)
        
        # 제어 전략 선택
        if corridor_width > 2.0:
            # 넓은 공간: 우측 벽 따라가기
            error = self.calculate_wall_following_error(left_dist, right_dist)
            strategy = "wall_following"
        else:
            # 좁은 복도: 중앙 유지
            error = center_error
            strategy = "center_following"
        
        # 조향각 계산
        steering_angle = self.calculate_steering_angle(error, dt)
        
        # 속도 계산
        velocity = self.calculate_adaptive_speed(front_dist, left_dist, right_dist, corridor_width)
        
        # 제어 명령 전송
        drive_msg = drive_param()
        drive_msg.angle = float(steering_angle)
        drive_msg.velocity = float(velocity)
        self.drive_publisher.publish(drive_msg)
        
        # 주기적 상태 로그
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 50 == 0:  # 5초마다
            self.get_logger().info(
                f'{strategy} | 전방:{front_dist:.2f} 좌:{left_dist:.2f} 우:{right_dist:.2f} | '
                f'폭:{corridor_width:.2f} 오차:{error:.3f} | 조향:{steering_angle:.3f} 속도:{velocity:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    wall_follower = StableWallFollower()
    
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        wall_follower.get_logger().info('키보드 인터럽트로 종료')
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()