# Move-to-Center Supervised Learning System

move_to_center 알고리즘을 teacher로 사용하는 supervised learning 시스템입니다.

## 📁 파일 구조

```
move_to_center_supervised/
├── move_to_center.py              # 원본 move_to_center 알고리즘
├── supervised_data_collector.py   # 데이터 수집기 (메인)
├── test_data_collector.py         # 데이터 분석 도구
└── README.md                      # 이 파일
```

## 🚀 사용법

### 1. 데이터 수집

```bash
# 기본 수집 (HDF5 + 압축, 속도 40, 5분)
python supervised_data_collector.py

# 다양한 포맷 옵션
python supervised_data_collector.py --format hdf5 --speed 50 --duration 180
python supervised_data_collector.py --format npz --no-compression
python supervised_data_collector.py --format pickle --speed 60
```

#### 파라미터:
- `--speed`: 목표 속도 (기본값: 40.0)
- `--duration`: 수집 시간 초 (기본값: 300)
- `--output-dir`: 출력 디렉토리 (기본값: ./collected_data)
- `--format`: 데이터 포맷 (hdf5/npz/pickle, 기본값: hdf5)
- `--no-compression`: 압축 비활성화

### 2. 데이터 분석

```bash
python test_data_collector.py
```

수집된 데이터의 통계, 품질, 시각화를 제공합니다.

### 3. 압축 성능 벤치마크

```bash
python compression_benchmark.py
```

다양한 압축 포맷의 크기와 속도를 비교합니다.

## 📊 데이터 구조

### HDF5/NPZ 포맷:
```python
{
    'timestamps': np.array,      # 타임스탬프 배열
    'lidar_data': np.array,      # LiDAR 스캔 (samples × 360)
    'steering_data': np.array,   # 조향각 배열
    'speed_data': np.array,      # 속도 배열
    'metadata': json_string      # 메타데이터 (JSON)
}
```

### 압축 효과:
- **Pickle**: 70MB (기준)
- **Pickle + gzip**: 15-20MB (3-4x 압축)
- **HDF5 + gzip**: 10-15MB (4-7x 압축)
- **HDF5 + Float16**: 5-8MB (8-14x 압축)

## 🛡️ 데이터 품질 관리

자동으로 다음 데이터를 필터링합니다:
- 급격한 조향 변화 (>0.2 rad)
- 너무 가까운/먼 벽 거리
- 극단적인 조향각 (>0.25 rad)

## 📈 다음 단계

1. **신경망 모델 구현** (`f1_neural_network.py`)
2. **학습기 구현** (`supervised_trainer.py`)
3. **ML 드라이버 구현** (`ml_f1_driver.py`)
4. **Curriculum 컨트롤러** (`curriculum_controller.py`)

## 🎯 Curriculum Learning 계획

```
Stage 1: 속도 40  → 안정적인 기본 데이터
Stage 2: 속도 50  → 중속 주행 학습
Stage 3: 속도 60  → 고속 주행 도전
Stage 4: 속도 80+ → 레이싱 성능
```

## 💡 팁

- 첫 수집은 짧게 (60초) 테스트해보세요
- 데이터 품질을 `test_data_collector.py`로 확인하세요
- 다양한 트랙 구간에서 수집하면 더 좋습니다
- 30초마다 자동 중간저장되므로 안전합니다