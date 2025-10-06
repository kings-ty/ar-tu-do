# 🏎️ Advanced T-Pattern LiDAR Racing Controller

ROS2 기반 F1 자율주행 시스템으로, T패턴 라이다 스캐닝과 적응적 속도 제어를 구현한 고성능 레이싱 컨트롤러입니다.

## 🚀 핵심 기술

### 1. **T패턴 라이다 스캐닝 시스템**
- **11개 포인트 동시 스캔**: 전방 5개 + 좌우 6개 각도
- **실시간 코너 예측**: 전방 1-4m 거리로 코너 미리 감지
- **RViz 시각화**: 색상별 마커와 레이저 빔 실시간 표시

```python
T패턴 구성:
전방(세로): 0°, ±5°, ±10° (코너 감지)
좌우(가로): ±90°, ±60°, ±45° (벽 추종)
```

### 2. **적응적 속도 제어 알고리즘**
- **4단계 자동 속도 조절**
  - 🟢 **장직선** (4m+): 최대 60km/h
  - 🔵 **일반직선** (2-4m): 기본 40km/h의 80-120%
  - 🟡 **코너** (1.2-2m): 20-32km/h
  - 🔴 **급코너** (1.2m 미만): 최소 15km/h

### 3. **멀티모달 센서 융합**
- **라이다**: T패턴 거리 측정 + 벽 추종
- **카메라**: 랩 체크 라인 감지 (OpenCV)
- **IMU**: 차량 자세 데이터

### 4. **Ackermann 조향 기하학**
- 물리적으로 정확한 앞바퀴 차동 조향각 계산
- 휠베이스/트랙폭 기반 조향 안정성 최적화

## 📊 실시간 모니터링

터미널에서 실시간으로 확인할 수 있는 정보:

```
============================================================
🎯 T패턴 라이다 분석:
전방 스캔: 0°=2.33m, ±5°=(1.96, 2.86)m, ±10°=(1.68, 3.72)m
좌우 스캔: 90°=0.72m, -90°=0.68m
현재 트랙 폭: 1.40m
🔄 코너 감지! (전방 최소거리: 1.96m)
랩 체크 라인: ❌ 없음
🚗 속도 제어: 코너 | 전방거리: 1.68m | 속도: 27.2 (68.1%)
============================================================
```

## 🛠️ 설치 및 실행

### 환경 요구사항
- ROS2 (Humble/Foxy)
- Python 3.8+
- OpenCV 4.x
- NumPy, SciPy

### 실행 방법

1. **시뮬레이션 환경 시작**
```bash
# Gazebo + RViz 실행
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

2. **T패턴 컨트롤러 실행**
```bash
cd /home/ty/f1test_ws/ar-tu-do
source install/setup.bash
python3 ros_ws/src/autonomous/move_to_center_supervised/move_to_center.py
```

3. **RViz 시각화 설정**
- Fixed Frame: `base_link`
- LaserScan Topic: `/scan` (Size: 0.05)
- MarkerArray Topic: `/t_pattern_markers` (T패턴 시각화)

## 🎯 RViz 시각화

### 마커 색상 의미
- 🔴 **빨간 구**: 정면 0도 (가장 중요)
- 🟠 **주황 구**: 전방 ±5°, ±10° (코너 감지용)
- 🔵 **파란 구**: 좌우 ±90° (벽 거리)
- 🟢 **녹색 구**: 기타 각도 (±45°, ±60°)
- 회색 선: 차량에서 각 포인트까지의 레이저 빔

## ⚙️ 주요 파라미터

```python
# 속도 제어 설정
BASE_SPEED = 40.0          # 기본 속도 (km/h)
MAX_SPEED = 60.0           # 최대 속도 (직선)
MIN_SPEED = 15.0           # 최소 속도 (급코너)

# T패턴 거리 임계값
STRAIGHT_DISTANCE = 4.0    # 직선 판단 거리
CORNER_DISTANCE = 2.0      # 코너 판단 거리
SHARP_CORNER_DISTANCE = 1.2 # 급코너 판단 거리

# 차량 물리 파라미터
WHEEL_BASE = 0.32          # 휠베이스 (m)
WHEEL_TRACK = 0.211        # 트랙폭 (m)
MAX_STEERING_ANGLE = 0.3   # 최대 조향각 (rad)
```

## 📈 성능 특징

### 속도 프로파일
- **직선 구간**: 최대 60km/h로 고속 주행
- **코너 진입**: 거리 기반 점진적 감속
- **급코너**: 15km/h까지 안전 감속
- **랩 라인**: 자동 30% 감속

### 안전성 기능
- 실시간 전방 장애물 감지
- 좌우 균형도 기반 속도 보정
- 물리적 조향 각도 제한
- 벽 충돌 방지 알고리즘

## 🔧 고급 기능

### 1. **데이터 수집 모드**
```bash
python supervised_data_collector.py --speed 50 --duration 300
```

### 2. **성능 분석 도구**
```bash
python test_data_collector.py  # 데이터 품질 분석
python compression_benchmark.py  # 압축 성능 테스트
```

### 3. **신경망 학습** (개발 중)
```bash
python supervised_trainer.py  # 딥러닝 모델 학습
python f1_neural_network.py   # 신경망 추론 모드
```

## 🏁 알고리즘 상세

### T패턴 레이캐스팅
```python
def calculate_adaptive_speed(self, t_data):
    # 전방 5개 각도에서 거리 측정
    forward_distances = [t_data[angle] for angle in [0, ±5, ±10]]
    min_distance = min(forward_distances)
    
    # 거리 기반 속도 계산
    if min_distance >= 4.0:      # 장직선
        speed = MAX_SPEED * 1.5
    elif min_distance >= 2.0:    # 일반직선
        speed = BASE_SPEED * (0.8 + 0.4 * ratio)
    elif min_distance >= 1.2:    # 코너
        speed = BASE_SPEED * (0.5 + 0.3 * ratio)
    else:                        # 급코너
        speed = MAX(MIN_SPEED, BASE_SPEED * 0.4)
```

### Ackermann 조향
```python
def calculate_ackermann_steering(self, center_angle):
    angle_outer = atan(wheelbase / (wheelbase/tan(center_angle) + track_width))
    angle_inner = atan(wheelbase / (wheelbase/tan(center_angle) - track_width))
```

## 📝 로그 분석

실시간 성능 모니터링을 위한 로그 분석:
- 코너/직선 감지 정확도
- 속도 변화 패턴
- 조향각 안정성
- 랩 타임 성능

## 🎮 커스터마이징

속도와 주행 스타일을 원하는 대로 조정:

```python
# 공격적 레이싱 모드
BASE_SPEED = 60.0
MAX_SPEED = 80.0
CORNER_DISTANCE = 1.5

# 안전 주행 모드  
BASE_SPEED = 30.0
MAX_SPEED = 45.0
CORNER_DISTANCE = 3.0
```

## 📚 참고 자료

- [F1TENTH 공식 문서](https://f1tenth.org/)
- [ROS2 LaserScan 메시지](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [Ackermann 조향 기하학](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)

---

**🏆 Created by**: Advanced Autonomous Racing Team  
**📅 Updated**: 2025-10-06  
**🔧 ROS2**: Humble/Foxy Compatible  
**🎯 Purpose**: High-Performance Autonomous F1 Racing