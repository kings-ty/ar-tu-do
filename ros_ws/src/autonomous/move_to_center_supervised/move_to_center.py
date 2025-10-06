#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import math
import cv2

class MoveToCenterController(Node):
    def __init__(self):
        super().__init__('move_to_center_controller')
        
        # 카메라 브리지
        self.bridge = CvBridge()
        
        # LiDAR 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # 카메라 구독자
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        
        # Publishers - 이전 코드와 동일한 방식
        self.pub_lw = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.pub_rw = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.pub_ls = self.create_publisher(Float64MultiArray, '/left_steering_controller/commands', 10)
        self.pub_rs = self.create_publisher(Float64MultiArray, '/right_steering_controller/commands', 10)
        
        # T패턴 시각화를 위한 Marker Publisher
        self.t_pattern_pub = self.create_publisher(MarkerArray, '/t_pattern_markers', 10)
        
        # 제어 파라미터
        self.BASE_SPEED = 40.0      # 기본 속도
        self.MIN_SPEED = 15.0       # 최소 속도 (급코너)
        self.MAX_SPEED = 60.0       # 최대 속도 (직선)
        self.MAX_STEERING_ANGLE = 0.3
        self.WHEEL_BASE = 0.32
        self.WHEEL_TRACK = 0.211
        
        # T패턴 기반 속도 제어 파라미터
        self.STRAIGHT_DISTANCE = 4.0    # 직선으로 판단하는 최소 거리
        self.CORNER_DISTANCE = 2.0      # 코너로 판단하는 거리
        self.SHARP_CORNER_DISTANCE = 1.2  # 급코너로 판단하는 거리
        
        # 트랙 폭 (측정된 값)
        self.TRACK_WIDTH = 1.368  # 측정된 트랙 폭
        self.TARGET_CENTER_DISTANCE = self.TRACK_WIDTH / 2.0  # 0.684m
        
        # 랩 체크 라인 감지 변수
        self.lap_line_detected = False
        self.MIN_LAP_LINE_WIDTH = 1.0  # 최소 랩 라인 폭 (m)
        
        self.get_logger().info(f'중앙 이동 컨트롤러 시작 - 목표: 좌우 각각 {self.TARGET_CENTER_DISTANCE:.3f}m')
        self.get_logger().info(f'랩 체크 라인 최소 폭: {self.MIN_LAP_LINE_WIDTH}m')
    
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
    
    def detect_horizontal_lap_line(self, image):
        """가로 방향의 랩 체크 라인 감지"""
        try:
            # 그레이스케일 변환
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 가우시안 블러 적용
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 하얀색 선 감지를 위한 임계값 적용
            _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
            
            # 형태학적 연산으로 노이즈 제거
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            
            # 윤곽선 찾기
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 가로 라인 후보들 찾기
            horizontal_lines = []
            image_height, image_width = gray.shape
            
            for contour in contours:
                # 윤곽선의 경계 박스 구하기
                x, y, w, h = cv2.boundingRect(contour)
                
                # 가로 라인 조건 검사
                aspect_ratio = w / h if h > 0 else 0
                
                # 1. 가로세로 비율이 충분히 큰지 (가로로 길어야 함)
                # 2. 높이가 적당한지 (너무 두껍지 않아야 함)
                # 3. 폭이 충분한지
                if (aspect_ratio > 10 and  # 가로가 세로의 10배 이상
                    h < image_height * 0.1 and  # 높이가 이미지의 10% 미만
                    w > image_width * 0.5):  # 폭이 이미지의 50% 이상
                    
                    horizontal_lines.append({
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h,
                        'aspect_ratio': aspect_ratio
                    })
            
            return horizontal_lines
            
        except Exception as e:
            self.get_logger().error(f'라인 감지 오류: {e}')
            return []
    
    def convert_pixel_to_real_width(self, pixel_width, image_width):
        """픽셀 폭을 실제 거리로 변환"""
        # 카메라 시야각과 거리를 고려한 변환
        # 가정: 1.3m 트랙에서 이미지 폭이 전체 트랙을 커버한다면
        real_width = (pixel_width / image_width) * self.TRACK_WIDTH
        return real_width
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            # ROS 이미지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 가로 라인 감지
            horizontal_lines = self.detect_horizontal_lap_line(cv_image)
            
            # 랩 체크 라인 검증
            prev_detected = self.lap_line_detected
            self.lap_line_detected = False
            
            for line in horizontal_lines:
                real_width = self.convert_pixel_to_real_width(line['width'], cv_image.shape[1])
                
                if real_width >= self.MIN_LAP_LINE_WIDTH:
                    self.lap_line_detected = True
                    if not prev_detected:  # 새로 감지된 경우만 로그 출력
                        self.get_logger().info(f'🏁 랩 체크 라인 감지! 폭: {real_width:.2f}m')
                    break
            
            # 디버깅: 이미지 수신 확인 (1초마다 1번만)
            import time
            current_time = time.time()
            if not hasattr(self, 'last_debug_time'):
                self.last_debug_time = 0
            
            if current_time - self.last_debug_time > 1.0:
                self.get_logger().info(f'카메라 작동 중 - 감지된 가로선: {len(horizontal_lines)}개')
                self.last_debug_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')
    
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
    
    def publish_t_pattern_visualization(self, scan_msg, t_data):
        """T패턴을 Marker로 시각화하여 RViz에서 명확하게 표시"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = 'laser'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # T패턴 포인트들을 구와 선으로 시각화
        for angle_deg, distance in t_data.items():
            if distance is not None:
                angle_rad = math.radians(angle_deg)
                x = distance * math.cos(angle_rad)
                y = distance * math.sin(angle_rad)
                
                # 구 마커 생성
                marker = Marker()
                marker.header.frame_id = 'laser'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 't_pattern'
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # 위치 설정
                marker.pose.position.x = float(x)
                marker.pose.position.y = float(y)
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # 크기 설정
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                
                # 색상 설정 (각도별로 다른 색상)
                if angle_deg == 0:  # 정면 - 빨간색
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                elif abs(angle_deg) <= 10:  # 전방 ±10도 - 주황색
                    marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                elif abs(angle_deg) == 90:  # 좌우 90도 - 파란색
                    marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
                else:  # 기타 - 녹색
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                
                marker.lifetime.sec = 1  # 1초 후 자동 삭제
                marker_array.markers.append(marker)
                
                # 원점에서 포인트까지의 선 마커
                line_marker = Marker()
                line_marker.header.frame_id = 'laser'
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = 't_pattern_lines'
                line_marker.id = marker_id + 100
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                
                # 선의 점들 (원점 → T패턴 포인트)
                start_point = Point()
                start_point.x = 0.0
                start_point.y = 0.0
                start_point.z = 0.0
                
                end_point = Point()
                end_point.x = float(x)
                end_point.y = float(y)
                end_point.z = 0.0
                
                line_marker.points = [start_point, end_point]
                
                # 선 스타일
                line_marker.scale.x = 0.02  # 선 두께
                line_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)  # 회색 반투명
                line_marker.lifetime.sec = 1
                
                marker_array.markers.append(line_marker)
                marker_id += 1
        
        self.t_pattern_pub.publish(marker_array)
    
    def calculate_adaptive_speed(self, t_data):
        """T패턴 전방 거리 기반 적응적 속도 계산"""
        # 전방 스캔 데이터 수집
        forward_distances = []
        for angle in [0, 5, -5, 10, -10]:  # 전방 T패턴 각도들
            if angle in t_data and t_data[angle] is not None:
                forward_distances.append(t_data[angle])
        
        if not forward_distances:
            return self.BASE_SPEED  # 데이터 없으면 기본 속도
        
        # 전방 거리 분석
        min_distance = min(forward_distances)
        avg_distance = sum(forward_distances) / len(forward_distances)
        
        # 거리 기반 속도 계산
        if min_distance >= self.STRAIGHT_DISTANCE:
            # 장직선: 최고 속도
            speed_ratio = min(1.5, min_distance / self.STRAIGHT_DISTANCE)
            target_speed = min(self.MAX_SPEED, self.BASE_SPEED * speed_ratio)
            road_type = "장직선"
            
        elif min_distance >= self.CORNER_DISTANCE:
            # 일반 직선: 기본~높은 속도
            speed_ratio = 0.8 + 0.4 * ((min_distance - self.CORNER_DISTANCE) / 
                                      (self.STRAIGHT_DISTANCE - self.CORNER_DISTANCE))
            target_speed = self.BASE_SPEED * speed_ratio
            road_type = "직선"
            
        elif min_distance >= self.SHARP_CORNER_DISTANCE:
            # 일반 코너: 중간 속도
            speed_ratio = 0.5 + 0.3 * ((min_distance - self.SHARP_CORNER_DISTANCE) / 
                                       (self.CORNER_DISTANCE - self.SHARP_CORNER_DISTANCE))
            target_speed = self.BASE_SPEED * speed_ratio
            road_type = "코너"
            
        else:
            # 급코너: 최저 속도
            speed_ratio = max(0.4, min_distance / self.SHARP_CORNER_DISTANCE * 0.5)
            target_speed = max(self.MIN_SPEED, self.BASE_SPEED * speed_ratio)
            road_type = "급코너"
        
        # 좌우 균형도 고려 (코너에서 안정성 향상)
        left_dist = t_data.get(90)
        right_dist = t_data.get(-90)
        if left_dist and right_dist:
            balance_ratio = min(left_dist, right_dist) / max(left_dist, right_dist)
            if balance_ratio < 0.7:  # 불균형한 경우
                target_speed *= (0.8 + 0.2 * balance_ratio)  # 최대 20% 감속
        
        # 디버그 정보 출력 (1초마다)
        import time
        current_time = time.time()
        if not hasattr(self, 'last_speed_debug_time'):
            self.last_speed_debug_time = 0
        
        if current_time - self.last_speed_debug_time > 1.0:
            print(f"🚗 속도 제어: {road_type} | 전방거리: {min_distance:.2f}m | 속도: {target_speed:.1f} ({speed_ratio:.1%})")
            self.last_speed_debug_time = current_time
        
        return target_speed
    
    def scan_callback(self, msg):
        # T패턴 데이터 수집
        t_pattern_data = {
            # 세로줄 (I) - 전방 스캔
            0: self.get_distance_at_angle(msg, 0),      # 정면
            5: self.get_distance_at_angle(msg, 5),      # 우측 5도
            -5: self.get_distance_at_angle(msg, -5),    # 좌측 5도
            10: self.get_distance_at_angle(msg, 10),    # 우측 10도
            -10: self.get_distance_at_angle(msg, -10),  # 좌측 10도
            
            # 가로줄 (ㅡ) - 좌우 스캔  
            90: self.get_distance_at_angle(msg, 90),    # 왼쪽
            -90: self.get_distance_at_angle(msg, -90),  # 오른쪽
            60: self.get_distance_at_angle(msg, 60),    # 좌측 60도
            -60: self.get_distance_at_angle(msg, -60),  # 우측 60도
            45: self.get_distance_at_angle(msg, 45),    # 좌측 45도
            -45: self.get_distance_at_angle(msg, -45),  # 우측 45도
        }
        
        # T패턴 시각화 발행
        self.publish_t_pattern_visualization(msg, t_pattern_data)
        
        # 기존 제어용 좌우 90도 거리 측정
        right_distance = self.get_distance_at_angle(msg, -90)  # -90도가 실제 오른쪽
        left_distance = self.get_distance_at_angle(msg, 90)    # +90도가 실제 왼쪽
        
        if right_distance is None or left_distance is None:
            self.get_logger().warn('LiDAR 데이터 없음')
            self.publish_commands(0.0, 0.0, 0.0)
            return
        
        # T패턴 정보 출력 (1초마다)
        import time
        current_time = time.time()
        if not hasattr(self, 'last_t_debug_time'):
            self.last_t_debug_time = 0
        
        if current_time - self.last_t_debug_time > 1.0:
            print("="*60)
            print(f"🎯 T패턴 라이다 분석:")
            print(f"전방 스캔: 0°={t_pattern_data[0]:.2f}m, ±5°=({t_pattern_data[-5]:.2f}, {t_pattern_data[5]:.2f})m, ±10°=({t_pattern_data[-10]:.2f}, {t_pattern_data[10]:.2f})m")
            print(f"좌우 스캔: 90°={left_distance:.2f}m, -90°={right_distance:.2f}m")
            print(f"현재 트랙 폭: {left_distance + right_distance:.2f}m")
            
            # 코너 감지 로직
            forward_min = min([d for d in [t_pattern_data[0], t_pattern_data[5], t_pattern_data[-5]] if d is not None])
            if forward_min < 2.0:
                print(f"🔄 코너 감지! (전방 최소거리: {forward_min:.2f}m)")
            else:
                print(f"➡️  직선 구간 (전방 여유: {forward_min:.2f}m)")
            
            print(f"랩 체크 라인: {'🏁 감지됨' if self.lap_line_detected else '❌ 없음'}")
            self.last_t_debug_time = current_time
        
        # 중앙에서의 편차 계산
        left_error = left_distance - self.TARGET_CENTER_DISTANCE
        right_error = right_distance - self.TARGET_CENTER_DISTANCE
        
        # 전체 위치 오차 (음수: 왼쪽 치우침, 양수: 오른쪽 치우침)
        position_error = right_distance - left_distance
        
        # T패턴 기반 적응적 속도 계산
        speed = self.calculate_adaptive_speed(t_pattern_data)
        
        # 조향각 계산 (기존 로직 유지)
        if abs(position_error) < 0.05:
            steering_angle = 0.0
        elif position_error > 0:
            steering_angle = -min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
        else:
            steering_angle = min(abs(position_error) * 0.5, self.MAX_STEERING_ANGLE)
        
        # 랩 체크 라인 감지 시 속도 조정
        if self.lap_line_detected:
            speed *= 0.7  # 랩 라인에서 30% 감속
            steering_angle *= 0.8  # 조향각도 약간 완화
        
        # Ackermann 조향 계산 및 명령 발행
        left_steer, right_steer = self.calculate_ackermann_steering(steering_angle)
        self.publish_commands(speed, left_steer, right_steer)

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