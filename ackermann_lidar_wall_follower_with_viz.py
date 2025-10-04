#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
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

class AckermannLidarWallFollowerViz(Node):
    def __init__(self):
        super().__init__('ackermann_lidar_wall_follower_viz')
        
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
        
        # 시각화 발행자들
        self.marker_pub = self.create_publisher(MarkerArray, '/wall_following_viz', 10)
        self.left_dist_pub = self.create_publisher(Float32, '/debug/left_distance', 10)
        self.right_dist_pub = self.create_publisher(Float32, '/debug/right_distance', 10)
        self.error_pub = self.create_publisher(Float32, '/debug/control_error', 10)
        self.steering_pub = self.create_publisher(Float32, '/debug/steering_angle', 10)
        
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
        
        self.get_logger().info('Ackermann LiDAR Wall Follower with Visualization started')
    
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
    
    def publish_debug_data(self, left_dist, right_dist, error, steering_angle):
        """디버그 데이터 발행"""
        # Float32 메시지들
        left_msg = Float32()
        left_msg.data = float(left_dist)
        self.left_dist_pub.publish(left_msg)
        
        right_msg = Float32()
        right_msg.data = float(right_dist)
        self.right_dist_pub.publish(right_msg)
        
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)
        
        steering_msg = Float32()
        steering_msg.data = float(steering_angle)
        self.steering_pub.publish(steering_msg)
    
    def create_visualization_markers(self, left_dist, right_dist, error, steering_angle):
        """RViz 시각화 마커 생성"""
        marker_array = MarkerArray()
        
        # 현재 시간
        current_time = self.get_clock().now().to_msg()
        
        # 좌측 거리 마커 (파란색 화살표)
        left_marker = Marker()
        left_marker.header.frame_id = "laser"
        left_marker.header.stamp = current_time
        left_marker.ns = "wall_distances"
        left_marker.id = 0
        left_marker.type = Marker.ARROW
        left_marker.action = Marker.ADD
        left_marker.pose.position.x = 0.0
        left_marker.pose.position.y = 0.0
        left_marker.pose.position.z = 0.0
        left_marker.pose.orientation.x = 0.0
        left_marker.pose.orientation.y = 0.0
        left_marker.pose.orientation.z = math.sin(math.pi/4)  # 45도 회전
        left_marker.pose.orientation.w = math.cos(math.pi/4)
        left_marker.scale.x = min(left_dist, 5.0)  # 화살표 길이
        left_marker.scale.y = 0.1
        left_marker.scale.z = 0.1
        left_marker.color.a = 1.0
        left_marker.color.r = 0.0
        left_marker.color.g = 0.0
        left_marker.color.b = 1.0  # 파란색
        
        # 우측 거리 마커 (빨간색 화살표)
        right_marker = Marker()
        right_marker.header.frame_id = "laser"
        right_marker.header.stamp = current_time
        right_marker.ns = "wall_distances"
        right_marker.id = 1
        right_marker.type = Marker.ARROW
        right_marker.action = Marker.ADD
        right_marker.pose.position.x = 0.0
        right_marker.pose.position.y = 0.0
        right_marker.pose.position.z = 0.0
        right_marker.pose.orientation.x = 0.0
        right_marker.pose.orientation.y = 0.0
        right_marker.pose.orientation.z = math.sin(-math.pi/4)  # -45도 회전
        right_marker.pose.orientation.w = math.cos(-math.pi/4)
        right_marker.scale.x = min(right_dist, 5.0)  # 화살표 길이
        right_marker.scale.y = 0.1
        right_marker.scale.z = 0.1
        right_marker.color.a = 1.0
        right_marker.color.r = 1.0  # 빨간색
        right_marker.color.g = 0.0
        right_marker.color.b = 0.0
        
        # 조향 방향 마커 (노란색 화살표)
        steering_marker = Marker()
        steering_marker.header.frame_id = "laser"
        steering_marker.header.stamp = current_time
        steering_marker.ns = "steering"
        steering_marker.id = 2
        steering_marker.type = Marker.ARROW
        steering_marker.action = Marker.ADD
        steering_marker.pose.position.x = 1.0  # 차량 앞쪽
        steering_marker.pose.position.y = 0.0
        steering_marker.pose.position.z = 0.0
        # 조향각에 따른 회전
        steering_marker.pose.orientation.x = 0.0
        steering_marker.pose.orientation.y = 0.0
        steering_marker.pose.orientation.z = math.sin(steering_angle/2)
        steering_marker.pose.orientation.w = math.cos(steering_angle/2)
        steering_marker.scale.x = 1.0
        steering_marker.scale.y = 0.15
        steering_marker.scale.z = 0.15
        steering_marker.color.a = 1.0
        steering_marker.color.r = 1.0  # 노란색
        steering_marker.color.g = 1.0
        steering_marker.color.b = 0.0
        
        marker_array.markers = [left_marker, right_marker, steering_marker]
        self.marker_pub.publish(marker_array)
    
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
    
    def follow_walls(self, scan):
        """양쪽 벽 추종 (ar-tu-do 방식)"""
        try:
            left_wall = self.get_wall(scan, False)   # 좌측 벽
            right_wall = self.get_wall(scan, True)   # 우측 벽
            
            # 예측 거리에서의 좌우 벽까지의 거리 계산
            left_predicted_distance = left_wall.predict_distance(self.prediction_distance)
            right_predicted_distance = right_wall.predict_distance(self.prediction_distance)
            
            # 중앙 유지를 위한 오차 계산
            # 양수: 오른쪽 벽이 더 가까움 (왼쪽으로 조향 필요)
            # 음수: 왼쪽 벽이 더 가까움 (오른쪽으로 조향 필요)
            error = right_predicted_distance - left_predicted_distance
            
            # PID 제어
            self.integral += error
            derivative = error - self.prev_error
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            
            # 조향각 계산 (ar-tu-do 방식)
            steering_angle = math.atan(correction) * 2 / math.pi
            steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
            
            # 속도 계산 (조향각에 따라 감소)
            velocity = self.DRIVE_SPEED * (1 - max(0.0, abs(steering_angle) - 0.15))
            velocity = max(0.5, min(velocity, self.DRIVE_SPEED))
            
            # 디버그 데이터 발행
            self.publish_debug_data(left_predicted_distance, right_predicted_distance, error, steering_angle)
            
            # 시각화 마커 생성
            self.create_visualization_markers(left_predicted_distance, right_predicted_distance, error, steering_angle)
            
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
    node = AckermannLidarWallFollowerViz()
    
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