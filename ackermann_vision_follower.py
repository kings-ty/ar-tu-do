#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2

class AckermannVisionFollower(Node):
    def __init__(self):
        super().__init__('ackermann_vision_follower')
        self.bridge = CvBridge()

        # Subscriber to the raw camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        
        # Publishers - simple_keyboard와 동일한 방식
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)
        
        # simple_keyboard의 설정값들
        self.DRIVE_SPEED = 3.0  # 기본 속도 (조금 낮게)
        self.STEERING_ANGLE_MAX = 0.4
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        self.get_logger().info('Ackermann Vision Follower node has been started.')
        self.get_logger().info('Publishers: wheel/steering controllers')
    
    def calculate_ackermann_steering(self, steer_angle_center):
        """Ackermann 조향 기하학 계산 (simple_keyboard와 동일)"""
        import math
        
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
        """제어 명령 발행 (simple_keyboard와 동일한 방식)"""
        # 속도 명령 (Float64MultiArray 형태)
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

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = cv_image.shape
            
            import numpy as np
            
            # ===========================================
            # 1. ROI (관심영역) 설정 및 시각화
            # ===========================================
            image_center = width // 2
            roi_height = height // 3
            roi = cv_image[height - roi_height:height, :]
            
            # 원본 이미지에 ROI 영역 표시
            display_image = cv_image.copy()
            cv2.rectangle(display_image, (0, height - roi_height), (width, height), (0, 255, 0), 2)
            cv2.putText(display_image, "ROI", (10, height - roi_height - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # ===========================================
            # 2. 그레이스케일 변환 및 분석
            # ===========================================
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # 좌우 절반으로 나누어서 밝기 평균 계산
            left_half = gray_roi[:, :width//2]
            right_half = gray_roi[:, width//2:]
            
            left_brightness = np.mean(left_half)
            right_brightness = np.mean(right_half)
            
            # ===========================================
            # 3. 밝기 분석 시각화
            # ===========================================
            # 좌우 영역을 다른 색으로 표시
            roi_colored = cv2.cvtColor(gray_roi, cv2.COLOR_GRAY2BGR)
            
            # 좌측 영역 표시 (파란색 오버레이)
            left_overlay = roi_colored.copy()
            left_overlay[:, :width//2] = [255, 100, 100]  # 파란색 계열
            roi_colored[:, :width//2] = cv2.addWeighted(roi_colored[:, :width//2], 0.7, 
                                                       left_overlay[:, :width//2], 0.3, 0)
            
            # 우측 영역 표시 (빨간색 오버레이)
            right_overlay = roi_colored.copy()
            right_overlay[:, width//2:] = [100, 100, 255]  # 빨간색 계열
            roi_colored[:, width//2:] = cv2.addWeighted(roi_colored[:, width//2:], 0.7, 
                                                       right_overlay[:, width//2:], 0.3, 0)
            
            # 중앙선 표시
            cv2.line(roi_colored, (width//2, 0), (width//2, roi_height), (0, 255, 255), 2)
            
            # ===========================================
            # 4. 조향 결정 및 시각화
            # ===========================================
            brightness_diff = right_brightness - left_brightness
            
            # 조향각 계산 (정규화)
            steering_sensitivity = 0.3
            steer_angle = np.clip(brightness_diff * steering_sensitivity / 50.0, 
                                -self.STEERING_ANGLE_MAX, self.STEERING_ANGLE_MAX)
            
            # 조향 방향 화살표 표시
            arrow_start = (image_center, height - 50)
            arrow_length = int(steer_angle * 200)  # 조향각에 비례한 화살표 길이
            arrow_end = (image_center + arrow_length, height - 50)
            
            arrow_color = (0, 255, 0) if abs(steer_angle) < 0.1 else (0, 255, 255)
            cv2.arrowedLine(display_image, arrow_start, arrow_end, arrow_color, 3)
            
            # ===========================================
            # 5. 정보 텍스트 추가
            # ===========================================
            info_y = 30
            cv2.putText(display_image, f"Left Brightness: {left_brightness:.1f}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
            
            cv2.putText(display_image, f"Right Brightness: {right_brightness:.1f}", 
                       (10, info_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 2)
            
            cv2.putText(display_image, f"Brightness Diff: {brightness_diff:.1f}", 
                       (10, info_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_image, f"Steering Angle: {steer_angle:.3f}", 
                       (10, info_y + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # ===========================================
            # 6. 화면 출력
            # ===========================================
            cv2.imshow('Camera View with Analysis', display_image)
            cv2.imshow('ROI Analysis', roi_colored)
            cv2.imshow('Grayscale ROI', gray_roi)
            
            # 키 입력 처리 (ESC로 종료 가능)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC 키
                self.get_logger().info('ESC 키로 종료 요청')
                rclpy.shutdown()
            
            # ===========================================
            # 7. 제어 명령 발행
            # ===========================================
            # Ackermann 조향 계산
            left_steer, right_steer = self.calculate_ackermann_steering(steer_angle)
            
            # 명령 발행
            speed = self.DRIVE_SPEED
            self.publish_commands(speed, left_steer, right_steer)
            
            # 로그 (매 30프레임마다)
            if not hasattr(self, 'frame_count'):
                self.frame_count = 0
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Vision | 이미지:{width}x{height} 좌밝기:{left_brightness:.1f} 우밝기:{right_brightness:.1f} '
                    f'조향각:{steer_angle:.3f} 속도:{speed:.1f}'
                )

        except Exception as e:
            self.get_logger().error(f'이미지 처리 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AckermannVisionFollower()
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
        
        # OpenCV 창 정리
        cv2.destroyAllWindows()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
