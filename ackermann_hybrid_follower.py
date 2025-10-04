#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class AckermannHybridFollower(Node):
    def __init__(self):
        super().__init__('ackermann_hybrid_follower')
        self.bridge = CvBridge()

        # 구독자들
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        
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
        self.DRIVE_SPEED = 2.0
        self.MAX_STEERING_ANGLE = 0.4
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        # 센서 데이터 저장
        self.latest_scan = None
        self.lidar_available = False
        
        # PID 파라미터
        self.kp = 1.0
        self.prev_error = 0.0
        
        self.get_logger().info('Hybrid Follower started - Vision + LiDAR')
        
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
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [float(speed)]
        
        left_steer_msg = Float64MultiArray()
        left_steer_msg.data = [float(left_steer)]
        
        right_steer_msg = Float64MultiArray()
        right_steer_msg.data = [float(right_steer)]
        
        self.pub_lw.publish(vel_cmd)
        self.pub_rw.publish(vel_cmd)
        self.pub_ls.publish(left_steer_msg)
        self.pub_rs.publish(right_steer_msg)
    
    def scan_callback(self, msg):
        """LiDAR 데이터 수신"""
        self.latest_scan = msg
        self.lidar_available = True
        # LiDAR 데이터가 수신되면 로그 출력
        if not hasattr(self, 'lidar_logged'):
            self.get_logger().info('✅ LiDAR 데이터 수신 확인!')
            self.lidar_logged = True
    
    def get_lidar_distances(self):
        """LiDAR에서 좌우 거리 측정"""
        if not self.latest_scan:
            return None, None
        
        scan = self.latest_scan
        
        # 좌측 (90도) 및 우측 (-90도) 거리 측정
        left_angle = math.pi / 2  # 90도
        right_angle = -math.pi / 2  # -90도
        
        # 각도를 인덱스로 변환
        def angle_to_index(angle):
            normalized_angle = (angle - scan.angle_min) / (scan.angle_max - scan.angle_min)
            index = int(normalized_angle * len(scan.ranges))
            return max(0, min(len(scan.ranges) - 1, index))
        
        left_idx = angle_to_index(left_angle)
        right_idx = angle_to_index(right_angle)
        
        # 거리 값 추출 (여러 점의 평균)
        def get_avg_distance(center_idx, range_size=10):
            start_idx = max(0, center_idx - range_size // 2)
            end_idx = min(len(scan.ranges), center_idx + range_size // 2)
            
            valid_ranges = []
            for i in range(start_idx, end_idx):
                if scan.range_min <= scan.ranges[i] <= scan.range_max:
                    valid_ranges.append(scan.ranges[i])
            
            return np.mean(valid_ranges) if valid_ranges else float('inf')
        
        left_dist = get_avg_distance(left_idx)
        right_dist = get_avg_distance(right_idx)
        
        return left_dist, right_dist
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = cv_image.shape
            
            # 방법 선택: LiDAR 우선, Vision 백업
            steering_angle = 0.0
            control_method = "None"
            
            if self.lidar_available and self.latest_scan:
                # LiDAR 제어
                left_dist, right_dist = self.get_lidar_distances()
                
                if left_dist and right_dist and not (math.isinf(left_dist) or math.isinf(right_dist)):
                    # LiDAR 기반 제어
                    error = right_dist - left_dist  # 양수: 오른쪽이 멀음 (왼쪽으로 조향)
                    steering_angle = np.clip(error * 0.2, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
                    control_method = f"LiDAR(L:{left_dist:.1f} R:{right_dist:.1f})"
                else:
                    # LiDAR 데이터가 유효하지 않으면 Vision 사용
                    steering_angle = self.vision_control(cv_image)
                    control_method = "Vision(LiDAR invalid)"
            else:
                # LiDAR 없으면 Vision 사용
                steering_angle = self.vision_control(cv_image)
                control_method = "Vision(No LiDAR)"
            
            # 속도 계산
            speed = self.DRIVE_SPEED * (1.0 - abs(steering_angle) * 0.5)
            speed = max(0.5, speed)
            
            # 제어 명령 발행
            left_steer, right_steer = self.calculate_ackermann_steering(steering_angle)
            self.publish_commands(speed, left_steer, right_steer)
            
            # 로깅
            if not hasattr(self, 'frame_count'):
                self.frame_count = 0
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'{control_method} | 조향:{steering_angle:.3f} 속도:{speed:.1f}')
                
            # 시각화 (선택사항)
            cv2.imshow('Hybrid Control', cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Hybrid 제어 오류: {e}')
    
    def vision_control(self, cv_image):
        """비전 기반 제어 (백업용)"""
        height, width = cv_image.shape[:2]
        
        # ROI 설정
        roi_height = height // 3
        roi = cv_image[height - roi_height:height, :]
        
        # 그레이스케일 변환
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # 좌우 밝기 비교
        left_half = gray_roi[:, :width//2]
        right_half = gray_roi[:, width//2:]
        
        left_brightness = np.mean(left_half)
        right_brightness = np.mean(right_half)
        
        brightness_diff = right_brightness - left_brightness
        steering_angle = np.clip(brightness_diff * 0.01, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    node = AckermannHybridFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 종료')
    finally:
        # 정지 명령
        stop_cmd = Float64MultiArray()
        stop_cmd.data = [0.0]
        node.pub_lw.publish(stop_cmd)
        node.pub_rw.publish(stop_cmd)
        node.pub_ls.publish(stop_cmd)
        node.pub_rs.publish(stop_cmd)
        
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()