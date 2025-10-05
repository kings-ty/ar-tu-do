#!/usr/bin/env python3
"""
F1 자율주행을 위한 시계열 CNN+LSTM 신경망

아키텍처:
1. CNN: LiDAR 공간 특징 추출 (근거리/원거리 패턴)
2. LSTM: 시계열 특징 추출 (움직임 패턴, 트렌드)
3. Multi-head: 조향각과 속도를 별도로 예측

입력: LiDAR 시계열 (seq_len × lidar_points) + 속도 히스토리
출력: 조향각 + 목표 속도
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Tuple, Optional

class LiDARSpatialEncoder(nn.Module):
    """LiDAR 공간 특징 추출기 (1D CNN)"""
    
    def __init__(self, lidar_points: int = 1080, feature_dim: int = 128):
        super().__init__()
        
        self.lidar_points = lidar_points
        self.feature_dim = feature_dim
        
        # 다중 스케일 1D CNN
        # 근거리 세밀한 특징 + 원거리 넓은 패턴
        self.spatial_layers = nn.Sequential(
            # Layer 1: 세밀한 특징 (kernel=5)
            nn.Conv1d(1, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 1080 -> 540
            
            # Layer 2: 중간 특징 (kernel=9)
            nn.Conv1d(32, 64, kernel_size=9, padding=4),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 540 -> 270
            
            # Layer 3: 넓은 특징 (kernel=15)
            nn.Conv1d(64, 128, kernel_size=15, padding=7),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 270 -> 135
            
            # Layer 4: 전역 특징 (kernel=21)
            nn.Conv1d(128, 256, kernel_size=21, padding=10),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(32)  # 고정 크기로 압축
        )
        
        # 특징 압축
        self.feature_compress = nn.Sequential(
            nn.Linear(256 * 32, feature_dim * 2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(feature_dim * 2, feature_dim)
        )
        
    def forward(self, lidar_scan: torch.Tensor) -> torch.Tensor:
        """
        Args:
            lidar_scan: (batch, lidar_points) 
        Returns:
            features: (batch, feature_dim)
        """
        # (batch, lidar_points) -> (batch, 1, lidar_points)
        x = lidar_scan.unsqueeze(1)
        
        # CNN 특징 추출
        x = self.spatial_layers(x)  # (batch, 256, 32)
        
        # Flatten and compress
        x = x.flatten(1)  # (batch, 256*32)
        features = self.feature_compress(x)  # (batch, feature_dim)
        
        return features

class F1TemporalEncoder(nn.Module):
    """시계열 특징 추출기 (LSTM)"""
    
    def __init__(self, spatial_dim: int = 128, speed_dim: int = 1, 
                 hidden_size: int = 256, num_layers: int = 2):
        super().__init__()
        
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        # LSTM 입력: 공간특징 + 속도
        input_size = spatial_dim + speed_dim
        
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=0.2 if num_layers > 1 else 0,
            bidirectional=True  # 양방향으로 과거와 미래 맥락 모두 활용
        )
        
        # 양방향이므로 hidden_size * 2
        self.output_size = hidden_size * 2
        
    def forward(self, spatial_features: torch.Tensor, 
                speed_sequence: torch.Tensor) -> torch.Tensor:
        """
        Args:
            spatial_features: (batch, seq_len, spatial_dim)
            speed_sequence: (batch, seq_len, 1)
        Returns:
            temporal_features: (batch, seq_len, hidden_size*2)
        """
        # 공간 특징과 속도 결합
        combined = torch.cat([spatial_features, speed_sequence], dim=-1)
        
        # LSTM 처리
        lstm_out, _ = self.lstm(combined)
        
        return lstm_out

class F1DrivingNet(nn.Module):
    """F1 자율주행 신경망 (CNN + LSTM)"""
    
    def __init__(self, lidar_points: int = 1080, sequence_length: int = 5,
                 spatial_dim: int = 128, temporal_hidden: int = 256):
        super().__init__()
        
        self.lidar_points = lidar_points
        self.sequence_length = sequence_length
        self.spatial_dim = spatial_dim
        
        # 공간 특징 추출기 (CNN)
        self.spatial_encoder = LiDARSpatialEncoder(lidar_points, spatial_dim)
        
        # 시계열 특징 추출기 (LSTM)
        self.temporal_encoder = F1TemporalEncoder(
            spatial_dim=spatial_dim,
            speed_dim=1,
            hidden_size=temporal_hidden,
            num_layers=2
        )
        
        # 최종 특징 차원
        final_dim = temporal_hidden * 2  # 양방향 LSTM
        
        # 조향각 예측 헤드
        self.steering_head = nn.Sequential(
            nn.Linear(final_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Tanh()  # -1 ~ +1 범위
        )
        
        # 속도 예측 헤드  
        self.speed_head = nn.Sequential(
            nn.Linear(final_dim, 256),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Sigmoid()  # 0 ~ 1 범위 (나중에 스케일링)
        )
        
        # 어텐션 메커니즘 (중요한 시점 강조)
        self.attention = nn.Sequential(
            nn.Linear(final_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 1),
            nn.Softmax(dim=1)
        )
        
    def forward(self, lidar_sequence: torch.Tensor, 
                speed_sequence: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            lidar_sequence: (batch, seq_len, lidar_points)
            speed_sequence: (batch, seq_len, 1)
        Returns:
            steering: (batch, 1) - 조향각 (-1 ~ +1)
            speed: (batch, 1) - 정규화된 속도 (0 ~ 1)
        """
        batch_size, seq_len, _ = lidar_sequence.shape
        
        # 각 시점별로 공간 특징 추출
        spatial_features = []
        for t in range(seq_len):
            lidar_t = lidar_sequence[:, t]  # (batch, lidar_points)
            spatial_t = self.spatial_encoder(lidar_t)  # (batch, spatial_dim)
            spatial_features.append(spatial_t)
        
        # 시계열로 스택
        spatial_features = torch.stack(spatial_features, dim=1)  # (batch, seq_len, spatial_dim)
        
        # 시계열 특징 추출
        temporal_features = self.temporal_encoder(spatial_features, speed_sequence)
        # (batch, seq_len, hidden_size*2)
        
        # 어텐션으로 중요한 시점 강조
        attention_weights = self.attention(temporal_features)  # (batch, seq_len, 1)
        attended_features = (temporal_features * attention_weights).sum(dim=1)  # (batch, hidden_size*2)
        
        # 최종 예측
        steering = self.steering_head(attended_features)  # (batch, 1)
        speed = self.speed_head(attended_features)  # (batch, 1)
        
        return steering, speed
    
    def predict_single(self, lidar_sequence: np.ndarray, 
                      speed_sequence: np.ndarray, 
                      device: str = 'cpu') -> Tuple[float, float]:
        """단일 예측 (추론용)"""
        self.eval()
        with torch.no_grad():
            # NumPy -> Tensor
            lidar_tensor = torch.FloatTensor(lidar_sequence).unsqueeze(0).to(device)
            speed_tensor = torch.FloatTensor(speed_sequence).unsqueeze(0).unsqueeze(-1).to(device)
            
            # 예측
            steering, speed = self.forward(lidar_tensor, speed_tensor)
            
            return steering.item(), speed.item()

class F1NetworkUtils:
    """신경망 유틸리티 함수들"""
    
    @staticmethod
    def count_parameters(model: nn.Module) -> int:
        """모델 파라미터 수 계산"""
        return sum(p.numel() for p in model.parameters() if p.requires_grad)
    
    @staticmethod
    def model_summary(model: nn.Module, lidar_points: int = 1080, seq_len: int = 5):
        """모델 요약 정보"""
        total_params = F1NetworkUtils.count_parameters(model)
        
        # 샘플 입력으로 크기 확인
        sample_lidar = torch.randn(1, seq_len, lidar_points)
        sample_speed = torch.randn(1, seq_len, 1)
        
        model.eval()
        with torch.no_grad():
            steering, speed = model(sample_lidar, sample_speed)
        
        print("=== F1 Driving Network Summary ===")
        print(f"Total Parameters: {total_params:,}")
        print(f"Input Shape: LiDAR ({seq_len}, {lidar_points}), Speed ({seq_len},)")
        print(f"Output Shape: Steering {steering.shape}, Speed {speed.shape}")
        print(f"Model Size: ~{total_params * 4 / 1024 / 1024:.1f} MB (FP32)")
        
        return total_params

def create_f1_model(lidar_points: int = 1080, sequence_length: int = 5) -> F1DrivingNet:
    """F1 모델 생성 팩토리 함수"""
    model = F1DrivingNet(
        lidar_points=lidar_points,
        sequence_length=sequence_length,
        spatial_dim=128,
        temporal_hidden=256
    )
    
    # 가중치 초기화
    def init_weights(m):
        if isinstance(m, nn.Linear):
            torch.nn.init.xavier_uniform_(m.weight)
            if m.bias is not None:
                torch.nn.init.zeros_(m.bias)
        elif isinstance(m, nn.Conv1d):
            torch.nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
    
    model.apply(init_weights)
    
    return model

if __name__ == '__main__':
    # 테스트
    print("F1 Neural Network 테스트")
    
    # 모델 생성
    model = create_f1_model(lidar_points=1080, sequence_length=5)
    
    # 모델 요약
    F1NetworkUtils.model_summary(model)
    
    # 샘플 데이터로 테스트
    batch_size = 2
    seq_len = 5
    lidar_points = 1080
    
    sample_lidar = torch.randn(batch_size, seq_len, lidar_points)
    sample_speed = torch.randn(batch_size, seq_len, 1)
    
    print(f"\n=== 순전파 테스트 ===")
    print(f"입력 - LiDAR: {sample_lidar.shape}, 속도: {sample_speed.shape}")
    
    steering, speed = model(sample_lidar, sample_speed)
    
    print(f"출력 - 조향각: {steering.shape} {steering.squeeze()}")
    print(f"출력 - 속도: {speed.shape} {speed.squeeze()}")
    
    print("\n✅ 신경망 구조 검증 완료!")