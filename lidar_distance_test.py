#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarDistanceTest(Node):
    def __init__(self):
        super().__init__('lidar_distance_test')
        
        # LiDAR 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.get_logger().info('LiDAR 거리 측정 테스트 시작')
        self.get_logger().info('차량이 가만히 있어도 좌우 벽까지 거리를 측정합니다')
    
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
    
    def scan_callback(self, msg):
        print("="*50)
        print("LiDAR 스캔 정보:")
        print(f"총 스캔 포인트: {len(msg.ranges)}개")
        print(f"스캔 범위: {math.degrees(msg.angle_min):.1f}° ~ {math.degrees(msg.angle_max):.1f}°")
        print(f"각도 증분: {math.degrees(msg.angle_increment):.2f}°")
        
        # 여러 각도에서 거리 측정
        test_angles = [
            -90,  # 완전 왼쪽 (수평)
            -70,  # 왼쪽 대각선
            -50,  # 왼쪽 약간
            -30,  # 왼쪽 조금
            0,    # 정면
            30,   # 오른쪽 조금  
            50,   # 오른쪽 약간
            70,   # 오른쪽 대각선
            90    # 완전 오른쪽 (수평)
        ]
        
        print("\n각도별 거리 측정 (대각선 거리 → 수평 거리):")
        left_horizontal_distances = []
        right_horizontal_distances = []
        
        for angle in test_angles:
            diagonal_distance = self.get_distance_at_angle(msg, angle)
            if diagonal_distance is not None:
                # 수평 거리 계산: 대각선 거리 × |sin(각도)|
                # ±90°는 이미 수평이므로 그대로 사용
                if abs(angle) == 90:
                    horizontal_distance = diagonal_distance
                else:
                    horizontal_distance = diagonal_distance * abs(math.sin(math.radians(angle)))
                
                print(f"{angle:4.0f}°: {diagonal_distance:.3f}m (대각선) → {horizontal_distance:.3f}m (수평)")
                
                if angle < 0:  # 왼쪽
                    left_horizontal_distances.append(horizontal_distance)
                elif angle > 0:  # 오른쪽
                    right_horizontal_distances.append(horizontal_distance)
            else:
                print(f"{angle:4.0f}°: 측정 불가")
        
        # 좌우 최소 수평 거리 (가장 가까운 벽까지의 실제 거리)
        print("\n=== 수평 거리 기준 분석 ===")
        if left_horizontal_distances:
            min_left_horizontal = min(left_horizontal_distances)
            print(f"좌측 벽까지 최소 수평거리: {min_left_horizontal:.3f}m")
        
        if right_horizontal_distances:
            min_right_horizontal = min(right_horizontal_distances)
            print(f"우측 벽까지 최소 수평거리: {min_right_horizontal:.3f}m")
        
        if left_horizontal_distances and right_horizontal_distances:
            total_width = min_left_horizontal + min_right_horizontal
            print(f"실제 트랙 폭: {total_width:.3f}m")
            print(f"차량 위치: 좌측벽에서 {min_left_horizontal:.3f}m, 우측벽에서 {min_right_horizontal:.3f}m")
            
            # 중앙 기준 편차
            center_offset = min_right_horizontal - min_left_horizontal
            if abs(center_offset) < 0.1:
                print("차량이 거의 중앙에 위치")
            elif center_offset > 0:
                print(f"차량이 왼쪽으로 {center_offset:.3f}m 치우침")
            else:
                print(f"차량이 오른쪽으로 {abs(center_offset):.3f}m 치우침")
        
        # 추가: 정확한 90도 측정값 확인
        left_90_distance = self.get_distance_at_angle(msg, -90)
        right_90_distance = self.get_distance_at_angle(msg, 90)
        
        print(f"\n=== 정확한 좌우 수평 거리 (90°) ===")
        if left_90_distance is not None:
            print(f"좌측 벽 (90°): {left_90_distance:.3f}m")
        if right_90_distance is not None:
            print(f"우측 벽 (90°): {right_90_distance:.3f}m")
        if left_90_distance is not None and right_90_distance is not None:
            exact_track_width = left_90_distance + right_90_distance
            print(f"정확한 트랙 폭: {exact_track_width:.3f}m")
        
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('테스트 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()