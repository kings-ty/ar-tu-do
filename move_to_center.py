#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class MoveToCenterController(Node):
    def __init__(self):
        super().__init__('move_to_center_controller')
        
        # LiDAR 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publishers - 이전 코드와 동일한 방식
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)
        
        # 제어 파라미터
        self.DRIVE_SPEED = 40.0  # 속도 올림
        self.MAX_STEERING_ANGLE = 0.3
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        # 트랙 폭 (측정된 값)
        self.TRACK_WIDTH = 1.368  # 측정된 트랙 폭
        self.TARGET_CENTER_DISTANCE = self.TRACK_WIDTH / 2.0  # 0.684m
        
        self.get_logger().info(f'중앙 이동 컨트롤러 시작 - 목표: 좌우 각각 {self.TARGET_CENTER_DISTANCE:.3f}m')
    
    def get_distance_at_angle(self, scan, angle_degrees):
        """특정 각도에서의 거리 측정"""
        angle_rad = math.radians(angle_degrees)
        
        # 스캔 범위 내 샘플 수 계산
        sample_count = len(scan.ranges)
        
        # 각도를 인덱스로 변환
        if scan.angle_max != scan.angle_min:
            index = int((angle_rad - scan.angle_min) / (scan.angle_max - scan.angle_min) * sample_count)
        else:
            return None
        
        # 범위 검사
        if 0 <= index < sample_count:
            distance = scan.ranges[index]
            if scan.range_min <= distance <= scan.range_max:
                return distance
        
        return None
    
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
    
    def scan_callback(self, msg):
        # 좌우 90도 거리 측정 (좌우 반대 수정)
        right_distance = self.get_distance_at_angle(msg, -90)  # -90도가 실제 오른쪽
        left_distance = self.get_distance_at_angle(msg, 90)    # +90도가 실제 왼쪽
        
        if right_distance is None or left_distance is None:
            self.get_logger().warn('LiDAR 데이터 없음')
            self.publish_commands(0.0, 0.0, 0.0)
            return
        
        print("="*50)
        print(f"현재 위치:")
        print(f"왼쪽 벽까지:  {left_distance:.3f}m (목표: {self.TARGET_CENTER_DISTANCE:.3f}m)")
        print(f"오른쪽 벽까지: {right_distance:.3f}m (목표: {self.TARGET_CENTER_DISTANCE:.3f}m)")
        print(f"현재 트랙 폭: {left_distance + right_distance:.3f}m")
        
        # 중앙에서의 편차 계산
        left_error = left_distance - self.TARGET_CENTER_DISTANCE
        right_error = right_distance - self.TARGET_CENTER_DISTANCE
        
        # 전체 위치 오차 (음수: 왼쪽 치우침, 양수: 오른쪽 치우침)
        position_error = right_distance - left_distance
        
        print(f"위치 오차: {position_error:.3f}m", end="")
        if abs(position_error) < 0.05:
            print(" (중앙에 위치! - 직진 모드)")
            steering_angle = 0.0
            speed = self.DRIVE_SPEED  # 중앙에서는 직진으로 계속 이동
        elif position_error > 0:
            print(" (오른쪽으로 치우침 - 왼쪽으로 이동 필요)")
            steering_angle = -min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
            speed = self.DRIVE_SPEED
        else:
            print(" (왼쪽으로 치우침 - 오른쪽으로 이동 필요)")
            steering_angle = min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
            speed = self.DRIVE_SPEED
        
        print(f"조향각: {steering_angle:.3f}rad, 속도: {speed:.1f}m/s")
        
        # Ackermann 조향 계산 및 명령 발행
        left_steer, right_steer = self.calculate_ackermann_steering(steering_angle)
        self.publish_commands(speed, left_steer, right_steer)
        
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToCenterController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('중앙 이동 컨트롤러 종료')
        # 정지 명령 발행
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0]
        node.pub_lw.publish(stop_cmd)
        node.pub_rw.publish(stop_cmd)
        node.pub_ls.publish(stop_cmd)
        node.pub_rs.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()