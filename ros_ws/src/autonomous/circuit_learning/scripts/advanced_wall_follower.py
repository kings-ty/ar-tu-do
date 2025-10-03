#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import numpy as np
import math
from collections import deque

class AdvancedWallFollower(Node):
    def __init__(self):
        super().__init__('advanced_wall_follower')
        
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
        
        # === 기본 제어 파라미터 ===
        self.target_wall_distance = 0.7  # 벽으로부터 목표 거리
        self.max_velocity = 0.5
        self.min_velocity = 0.1
        self.max_steering = 1.0
        
        # === PID 제어기 파라미터 ===
        self.kp = 3.0
        self.ki = 0.2
        self.kd = 0.8
        
        # === 적응적 제어 파라미터 ===
        self.corner_detection_threshold = 0.3  # 코너 감지 임계값
        self.tight_corridor_width = 1.2  # 좁은 복도 기준
        self.wide_area_width = 3.0  # 넓은 공간 기준
        
        # === 안전 파라미터 ===
        self.emergency_distance = 0.25
        self.caution_distance = 0.5
        self.safe_distance = 1.0
        
        # === 상태 변수 ===
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.previous_time = self.get_clock().now()
        
        # === 스무딩을 위한 히스토리 ===
        self.distance_history = {
            'left': deque(maxlen=5),
            'right': deque(maxlen=5),
            'front': deque(maxlen=3)
        }
        
        # === 주행 모드 ===
        self.driving_mode = "wall_following"  # wall_following, center_keeping, emergency
        
        self.get_logger().info('고급 Wall Follower 시작')
        self.get_logger().info(f'목표 거리: {self.target_wall_distance}m, 최대속도: {self.max_velocity}m/s')
    
    def smooth_distances(self, distances):
        """거리 데이터 스무딩"""
        smoothed = {}
        for direction, distance in distances.items():
            self.distance_history[direction].append(distance)
            if len(self.distance_history[direction]) > 0:
                smoothed[direction] = np.mean(list(self.distance_history[direction]))
            else:
                smoothed[direction] = distance
        return smoothed
    
    def analyze_environment(self, ranges, angle_min, angle_max):
        """환경 분석 및 거리 측정"""
        total_points = len(ranges)
        ranges_array = np.array(ranges)
        ranges_array[np.isinf(ranges_array)] = 10.0  # inf 처리
        
        # 각도별 인덱스 계산
        center_idx = total_points // 2
        quarter_idx = total_points // 4
        
        # 구역별 거리 측정
        distances = {
            'front': np.min(ranges_array[center_idx-quarter_idx//2:center_idx+quarter_idx//2]),
            'left': np.min(ranges_array[0:quarter_idx]),
            'right': np.min(ranges_array[-quarter_idx:]),
            'front_left': np.min(ranges_array[quarter_idx:center_idx]),
            'front_right': np.min(ranges_array[center_idx:total_points-quarter_idx])
        }
        
        # 거리 스무딩
        key_distances = {k: v for k, v in distances.items() if k in ['front', 'left', 'right']}
        smoothed = self.smooth_distances(key_distances)
        distances.update(smoothed)
        
        # 환경 특성 분석
        corridor_width = distances['left'] + distances['right']
        wall_balance = min(distances['left'], distances['right']) / max(distances['left'], distances['right'])
        
        # 코너 감지
        is_corner = self.detect_corner(distances)
        
        return distances, corridor_width, wall_balance, is_corner
    
    def detect_corner(self, distances):
        """코너 감지 알고리즘"""
        # 전방과 좌우 거리 비교로 코너 감지
        front_to_side_ratio = distances['front'] / max(distances['left'], distances['right'])
        
        # 좌우 불균형으로 코너 감지
        left_right_ratio = min(distances['left'], distances['right']) / max(distances['left'], distances['right'])
        
        return (front_to_side_ratio < 0.6) or (left_right_ratio < 0.4)
    
    def select_driving_strategy(self, distances, corridor_width, wall_balance, is_corner):
        """주행 전략 선택"""
        if distances['front'] < self.emergency_distance:
            return "emergency_stop"
        elif distances['front'] < self.caution_distance:
            return "emergency_avoidance"
        elif is_corner:
            return "corner_navigation"
        elif corridor_width < self.tight_corridor_width:
            return "center_keeping"
        elif corridor_width > self.wide_area_width:
            return "wall_following"
        else:
            return "adaptive_following"
    
    def calculate_error_by_strategy(self, strategy, distances, corridor_width):
        """전략별 오차 계산"""
        if strategy == "emergency_stop":
            return 0.0
        elif strategy == "emergency_avoidance":
            # 장애물 회피: 더 넓은 쪽으로 조향
            if distances['left'] > distances['right']:
                return -0.5  # 왼쪽으로
            else:
                return 0.5   # 오른쪽으로
        elif strategy == "center_keeping":
            # 복도 중앙 유지
            center_offset = (distances['left'] - distances['right']) / 2.0
            return -center_offset  # 중앙으로 향하도록 반전
        elif strategy == "wall_following":
            # 우측 벽 따라가기
            return distances['right'] - self.target_wall_distance
        elif strategy == "corner_navigation":
            # 코너에서는 안전거리 유지하며 중앙 선호
            if distances['front'] < 1.0:
                # 전방이 가까우면 더 넓은 쪽으로
                return (distances['left'] - distances['right']) * 0.3
            else:
                # 일반적인 벽 따라가기
                return distances['right'] - self.target_wall_distance
        else:  # adaptive_following
            # 상황에 따른 적응적 제어
            if distances['right'] < 0.4:  # 우측이 너무 가까우면
                return (distances['left'] - distances['right']) * 0.2
            else:
                return distances['right'] - self.target_wall_distance
    
    def calculate_adaptive_speed(self, strategy, distances, corridor_width, wall_balance):
        """전략별 적응적 속도 계산"""
        base_speed = self.max_velocity
        
        # 전략별 기본 속도 조정
        if strategy == "emergency_stop":
            return 0.0
        elif strategy == "emergency_avoidance":
            base_speed = self.min_velocity
        elif strategy == "corner_navigation":
            base_speed = self.max_velocity * 0.6
        elif strategy == "center_keeping":
            base_speed = self.max_velocity * 0.8
        
        # 거리별 속도 조정
        front_factor = 1.0
        if distances['front'] < self.safe_distance:
            front_factor = max(0.3, distances['front'] / self.safe_distance)
        
        # 복도 폭별 속도 조정
        width_factor = min(1.0, corridor_width / 2.0)
        
        # 벽 균형별 속도 조정
        balance_factor = 0.7 + 0.3 * wall_balance
        
        # 최종 속도 계산
        final_speed = base_speed * front_factor * width_factor * balance_factor
        return max(self.min_velocity, min(final_speed, self.max_velocity))
    
    def calculate_steering_with_pid(self, error, dt):
        """PID 제어로 조향각 계산"""
        if dt <= 0:
            return 0.0
        
        # PID 계산
        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)  # 적분 포화 방지
        
        derivative = (error - self.previous_error) / dt
        
        steering = (self.kp * error + 
                   self.ki * self.integral_error + 
                   self.kd * derivative)
        
        self.previous_error = error
        return np.clip(steering, -self.max_steering, self.max_steering)
    
    def laser_callback(self, msg):
        """메인 제어 로직"""
        # 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = current_time
        
        if dt <= 0:
            return
        
        # 환경 분석
        distances, corridor_width, wall_balance, is_corner = self.analyze_environment(
            msg.ranges, msg.angle_min, msg.angle_max
        )
        
        # 주행 전략 선택
        strategy = self.select_driving_strategy(distances, corridor_width, wall_balance, is_corner)
        self.driving_mode = strategy
        
        # 오차 계산
        error = self.calculate_error_by_strategy(strategy, distances, corridor_width)
        
        # 조향각 계산
        steering_angle = self.calculate_steering_with_pid(error, dt)
        
        # 속도 계산
        velocity = self.calculate_adaptive_speed(strategy, distances, corridor_width, wall_balance)
        
        # 제어 명령 전송
        drive_msg = drive_param()
        drive_msg.angle = float(steering_angle)
        drive_msg.velocity = float(velocity)
        self.drive_publisher.publish(drive_msg)
        
        # 로깅
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 30 == 0:  # 3초마다
            self.get_logger().info(
                f'[{strategy}] F:{distances["front"]:.2f} L:{distances["left"]:.2f} R:{distances["right"]:.2f} | '
                f'폭:{corridor_width:.2f} 균형:{wall_balance:.2f} | 오차:{error:.3f} 조향:{steering_angle:.3f} 속도:{velocity:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    wall_follower = AdvancedWallFollower()
    
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        wall_follower.get_logger().info('종료')
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()