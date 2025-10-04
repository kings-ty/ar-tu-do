#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class Wall:
    """ar-tu-do의 Wall 클래스를 파이썬으로 구현"""
    def __init__(self, angle1, angle2, range1, range2):
        self.angle1 = angle1
        self.angle2 = angle2
        self.range1 = range1
        self.range2 = range2
    
    def get_angle(self):
        """벽의 각도 계산 (ar-tu-do 방식)"""
        return math.atan((self.range1 * math.cos(abs(self.angle1 - self.angle2)) - self.range2) / 
                        (self.range1 * math.sin(abs(self.angle1 - self.angle2))))
    
    def predict_distance(self, distance_to_current_position):
        """미래 위치에서의 벽까지 거리 예측 (ar-tu-do 방식)"""
        angle = self.get_angle()
        current_wall_distance = self.range2 * math.cos(angle)
        return current_wall_distance + distance_to_current_position * math.sin(angle)

class AckermannLidarWallFollower(Node):
    def __init__(self):
        super().__init__('ackermann_lidar_wall_follower')
        
        # LiDAR 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # 제어 명령 발행자
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)
        
        # 제어 파라미터
        self.DRIVE_SPEED = 2.5
        self.MAX_STEERING_ANGLE = 0.4
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        # Wall following 파라미터 (ar-tu-do 방식)
        self.min_range = 0.2
        self.max_range = 30.0
        self.fallback_range = 2.0
        
        # 샘플링 각도 (라디안) - ar-tu-do 기본값
        self.sample_angle_1 = math.radians(50)   # 첫 번째 샘플 각도
        self.sample_angle_2 = math.radians(70)   # 두 번째 샘플 각도
        
        # 예측 거리 및 목표 거리
        self.prediction_distance = 1.0   # 전방 예측 거리
        self.target_wall_distance = 5.0  # 목표 벽 거리 (트랙 폭의 절반)
        
        # PID 제어 파라미터
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 안정화를 위한 히스토리 버퍼
        self.left_distance_history = []
        self.right_distance_history = []
        self.history_size = 10
        
        # 트랙 폭 동적 계산을 위한 변수들
        self.track_width_samples = []
        self.track_width_calibrated = False
        self.calibration_samples = 50  # 트랙 폭 보정을 위한 샘플 수
        self.target_center_distance = None  # 동적으로 계산될 값
        
        self.get_logger().info('Ackermann LiDAR Wall Follower (ar-tu-do style) started')
    
    def calculate_ackermann_steering(self, steer_angle_center):
        """Ackermann 조향 기하학 계산"""
        if steer_angle_center != 0.0:
            angle_outer = math.atan(self.WHEEL_BASE / (self.WHEEL_BASE / math.tan(abs(steer_angle_center)) + self.WHEEL_TRACK))
            angle_inner = math.atan(self.WHEEL_BASE / (self.WHEEL_BASE / math.tan(abs(steer_angle_center)) - self.WHEEL_TRACK))
            
            if steer_angle_center > 0:
                left_steer_cmd = angle_inner
                right_steer_cmd = angle_outer
            else:
                left_steer_cmd = -angle_outer
                right_steer_cmd = -angle_inner
        else:
            left_steer_cmd = 0.0
            right_steer_cmd = 0.0
        
        return left_steer_cmd, right_steer_cmd
    
    def publish_commands(self, speed, left_steer, right_steer):
        """제어 명령 발행"""
        # 속도 명령
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [float(speed)]
        
        # 조향 명령
        left_steer_msg = Float64MultiArray()
        left_steer_msg.data = [float(left_steer)]
        
        right_steer_msg = Float64MultiArray()
        right_steer_msg.data = [float(right_steer)]
        
        # 발행
        self.pub_lw.publish(vel_cmd)
        self.pub_rw.publish(vel_cmd)
        self.pub_ls.publish(left_steer_msg)
        self.pub_rs.publish(right_steer_msg)
    
    def get_range_at_angle(self, scan, angle):
        """특정 각도에서의 거리 값 가져오기 (ar-tu-do 방식)"""
        # 스캔 범위 내 샘플 수 계산
        sample_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        
        # 각도를 인덱스로 변환
        if sample_count > 0:
            index = int((angle - scan.angle_min) / (scan.angle_max - scan.angle_min) * sample_count)
        else:
            return self.fallback_range
        
        # 범위 및 유효성 검사
        if (index < 0 or 
            index >= sample_count or 
            index >= len(scan.ranges) or
            scan.ranges[index] < self.min_range or 
            scan.ranges[index] > self.max_range):
            self.get_logger().info("LiDAR 샘플링 실패, 대체값 사용")
            return self.fallback_range
        
        return scan.ranges[index]
    
    def get_wall(self, scan, right_wall):
        """Wall 객체 생성 (ar-tu-do 방식)"""
        left_right_sign = -1 if right_wall else 1
        
        angle1 = self.sample_angle_1 * left_right_sign
        angle2 = self.sample_angle_2 * left_right_sign
        range1 = self.get_range_at_angle(scan, angle1)
        range2 = self.get_range_at_angle(scan, angle2)
        
        return Wall(angle1, angle2, range1, range2)
    
    def calibrate_track_width(self, left_distance, right_distance):
        """트랙 폭 동적 보정"""
        if not self.track_width_calibrated:
            # 유효한 거리 값만 수집 (음수나 너무 큰 값 제외)
            if left_distance > 0.1 and right_distance > 0.1 and left_distance < 10 and right_distance < 10:
                track_width = left_distance + right_distance
                self.track_width_samples.append(track_width)
                
                print(f"트랙폭 샘플 #{len(self.track_width_samples)}: L={left_distance:.3f} + R={right_distance:.3f} = {track_width:.3f}m")
                
                if len(self.track_width_samples) >= self.calibration_samples:
                    # 이상치 제거 후 평균 계산
                    sorted_samples = sorted(self.track_width_samples)
                    print(f"\n=== 트랙 폭 계산 상세 ===")
                    print(f"전체 샘플: {len(sorted_samples)}개")
                    print(f"최소값: {min(sorted_samples):.3f}m")
                    print(f"최대값: {max(sorted_samples):.3f}m")
                    print(f"전체 평균: {np.mean(sorted_samples):.3f}m")
                    
                    # 상위/하위 25% 제거
                    trim_count = len(sorted_samples) // 4
                    trimmed_samples = sorted_samples[trim_count:-trim_count] if trim_count > 0 else sorted_samples
                    
                    print(f"이상치 제거: 상위/하위 {trim_count}개씩 제거")
                    print(f"필터링된 샘플: {len(trimmed_samples)}개")
                    print(f"필터링된 범위: {min(trimmed_samples):.3f}m ~ {max(trimmed_samples):.3f}m")
                    
                    avg_track_width = np.mean(trimmed_samples)
                    track_width_std = np.std(trimmed_samples)
                    self.target_center_distance = avg_track_width / 2.0
                    self.track_width_calibrated = True
                    
                    print(f"최종 트랙 폭: {avg_track_width:.3f}m (표준편차: {track_width_std:.3f})")
                    print(f"목표 중앙거리: {self.target_center_distance:.3f}m")
                    print("======================\n")
                    
                    self.get_logger().info(f'트랙 폭 보정 완료: {avg_track_width:.3f}m, 목표 중앙거리: {self.target_center_distance:.3f}m')
        
        return self.track_width_calibrated

    def follow_walls(self, scan):
        """양쪽 벽 추종 - 동적 트랙 폭 계산 및 표준편차 기반 안정화"""
        try:
            left_wall = self.get_wall(scan, False)   # 좌측 벽
            right_wall = self.get_wall(scan, True)   # 우측 벽
            
            # 예측 거리에서의 좌우 벽까지의 거리 계산
            left_predicted_distance = left_wall.predict_distance(self.prediction_distance)
            right_predicted_distance = right_wall.predict_distance(self.prediction_distance)
            
            # 트랙 폭 보정
            is_calibrated = self.calibrate_track_width(left_predicted_distance, right_predicted_distance)
            
            if not is_calibrated:
                # 보정 중: 기본 wall following
                error = right_predicted_distance - left_predicted_distance
                correction_factor = 0.5
                is_stable = False
                
                print("="*30)
                print(f"보정중 L:{left_predicted_distance:.3f} R:{right_predicted_distance:.3f} 샘플:{len(self.track_width_samples)}/{self.calibration_samples}")
            else:
                # 히스토리 업데이트 (이동 평균용)
                self.left_distance_history.append(left_predicted_distance)
                self.right_distance_history.append(right_predicted_distance)
                if len(self.left_distance_history) > self.history_size:
                    self.left_distance_history.pop(0)
                    self.right_distance_history.pop(0)
                
                # 평균과 표준편차 계산
                if len(self.left_distance_history) >= 3:
                    left_mean = np.mean(self.left_distance_history)
                    right_mean = np.mean(self.right_distance_history)
                    left_std = np.std(self.left_distance_history)
                    right_std = np.std(self.right_distance_history)
                    
                    # 안정성 체크: 표준편차가 낮으면 안정된 상태
                    stability_threshold = 0.1
                    is_stable = (left_std < stability_threshold and right_std < stability_threshold)
                    
                    # 현재 트랙 중앙에서의 편차 계산
                    current_center = (left_mean + right_mean) / 2.0
                    ideal_center = self.target_center_distance
                    center_error = ideal_center - current_center
                    
                    # 좌우 균형 확인
                    left_target = self.target_center_distance
                    right_target = self.target_center_distance
                    left_error = left_mean - left_target
                    right_error = right_mean - right_target
                    
                    if is_stable and abs(center_error) < 0.1:
                        # 매우 안정된 상태: 미세 조정만
                        error = (right_error - left_error) * 0.5
                        correction_factor = 0.2
                    elif is_stable:
                        # 안정되었지만 중앙에서 벗어남: 중앙으로 복귀
                        error = center_error
                        correction_factor = 0.5
                    else:
                        # 불안정한 상태: 더 가까운 벽에서 멀어지기
                        if abs(left_error) > abs(right_error):
                            error = -left_error  # 왼쪽 벽이 가까우면 오른쪽으로
                        else:
                            error = right_error  # 오른쪽 벽이 가까우면 왼쪽으로
                        correction_factor = 1.0
                else:
                    # 초기 데이터 수집 중
                    error = right_predicted_distance - left_predicted_distance
                    correction_factor = 0.8
                    is_stable = False
                
                print("="*30)
                print(f"L:{left_predicted_distance:.3f} R:{right_predicted_distance:.3f} 목표:{self.target_center_distance:.3f} Stable:{is_stable}")
            
            # PID 제어
            self.integral += error
            derivative = error - self.prev_error
            correction = self.kp * error * correction_factor + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            
            # 조향각 계산 (ar-tu-do 방식)
            steering_angle = math.atan(correction) * 2 / math.pi
            steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
            
            # 속도 계산 (조향각에 따라 감소)
            velocity = self.DRIVE_SPEED * (1 - max(0.0, abs(steering_angle) - 0.15))
            velocity = max(0.5, min(velocity, self.DRIVE_SPEED))
            
            # 로깅 (매 30번째 콜백마다)
            if not hasattr(self, 'callback_count'):
                self.callback_count = 0
            self.callback_count += 1
            
            if self.callback_count % 30 == 0:
                self.get_logger().info(
                    f'Wall Following | 좌예측:{left_predicted_distance:.2f}m '
                    f'우예측:{right_predicted_distance:.2f}m 오차:{error:.3f} '
                    f'조향:{steering_angle:.3f} 속도:{velocity:.1f}'
                )
                # 제어 방향 확인용 로그
                if error > 0:
                    self.get_logger().info('→ 오른쪽 벽이 더 가까움, 왼쪽으로 조향')
                elif error < 0:
                    self.get_logger().info('→ 왼쪽 벽이 더 가까움, 오른쪽으로 조향')
                else:
                    self.get_logger().info('→ 중앙 위치')
            
            return velocity, steering_angle
            
        except Exception as e:
            self.get_logger().error(f'Wall following 계산 오류: {e}')
            return 0.0, 0.0
    
    def scan_callback(self, msg):
        try:
            # ar-tu-do 스타일 wall following
            velocity, steering_angle = self.follow_walls(msg)
            
            # Ackermann 조향 계산 및 명령 발행
            left_steer, right_steer = self.calculate_ackermann_steering(steering_angle)
            self.publish_commands(velocity, left_steer, right_steer)
            
        except Exception as e:
            self.get_logger().error(f'LiDAR 처리 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AckermannLidarWallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 종료')
    finally:
        # 정지 명령 발행
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0]
        node.pub_lw.publish(stop_cmd)
        node.pub_rw.publish(stop_cmd)
        node.pub_ls.publish(stop_cmd)
        node.pub_rs.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()