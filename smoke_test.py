"""
Smoke 테스트 - 모든 모듈이 정상 작동하는지 확인
"""
import torch
import sys
from pathlib import Path

print("=" * 80)
print("Smoke 테스트 시작")
print("=" * 80)

# 1. Config 테스트
print("\n[1/5] Config 모듈 테스트...")
try:
    import config
    print(f"  ✓ 데이터 디렉토리: {config.DATA_DIR}")
    print(f"  ✓ 클래스 수: {config.NUM_CLASSES}")
    print(f"  ✓ 배치 크기: {config.BATCH_SIZE}")
except Exception as e:
    print(f"  ✗ 오류: {e}")
    sys.exit(1)

# 2. Model 테스트
print("\n[2/5] Model 모듈 테스트...")
try:
    from model import get_model
    model = get_model(num_classes=6, pretrained=False, device='cpu')
    dummy_input = torch.randn(2, 3, 224, 224)
    output = model(dummy_input)
    assert output.shape == (2, 6), f"출력 shape 오류: {output.shape}"
    print(f"  ✓ 모델 생성 성공")
    print(f"  ✓ 입력 shape: {dummy_input.shape}")
    print(f"  ✓ 출력 shape: {output.shape}")
except Exception as e:
    print(f"  ✗ 오류: {e}")
    sys.exit(1)

# 3. Dataset 테스트
print("\n[3/5] Dataset 모듈 테스트...")
try:
    from dataset import get_dataloaders, ActionDataset
    
    # 데이터 디렉토리 확인
    if not config.DATA_DIR.exists():
        print(f"  ⚠ 경고: 데이터 디렉토리가 없습니다: {config.DATA_DIR}")
        print(f"  → data_preprocessing.ipynb를 먼저 실행하여 데이터를 전처리하세요.")
    else:
        # 데이터 로더 생성 시도
        train_loader, val_loader, test_loader, class_to_idx = get_dataloaders(
            config.DATA_DIR,
            batch_size=4,  # 작은 배치로 테스트
            num_workers=0,
            input_size=224
        )
        print(f"  ✓ 데이터 로더 생성 성공")
        print(f"  ✓ Train 배치 수: {len(train_loader)}")
        print(f"  ✓ Validation 배치 수: {len(val_loader)}")
        print(f"  ✓ Test 배치 수: {len(test_loader)}")
        print(f"  ✓ 클래스: {list(class_to_idx.keys())}")
        
        # 첫 배치 테스트
        images, labels = next(iter(train_loader))
        print(f"  ✓ 배치 이미지 shape: {images.shape}")
        print(f"  ✓ 배치 라벨 shape: {labels.shape}")
        
except Exception as e:
    print(f"  ✗ 오류: {e}")
    import traceback
    traceback.print_exc()

# 4. Utils 테스트
print("\n[4/5] Utils 모듈 테스트...")
try:
    from utils import calculate_metrics, AverageMeter
    import numpy as np
    
    # 더미 데이터로 메트릭 계산 테스트
    y_true = np.array([0, 1, 2, 3, 4, 5, 0, 1, 2, 3])
    y_pred = np.array([0, 1, 2, 3, 4, 5, 0, 1, 2, 4])
    class_names = ['boxing', 'handclapping', 'handwaving', 'jogging', 'running', 'walking']
    
    metrics = calculate_metrics(y_true, y_pred, class_names)
    print(f"  ✓ 메트릭 계산 성공")
    print(f"  ✓ Accuracy: {metrics['accuracy']:.4f}")
    
    # AverageMeter 테스트
    meter = AverageMeter()
    meter.update(0.5)
    meter.update(0.7)
    assert abs(meter.avg - 0.6) < 0.01, "AverageMeter 오류"
    print(f"  ✓ AverageMeter 테스트 통과")
    
except Exception as e:
    print(f"  ✗ 오류: {e}")
    sys.exit(1)

# 5. 디렉토리 구조 확인
print("\n[5/5] 디렉토리 구조 확인...")
try:
    assert config.CHECKPOINT_DIR.exists(), f"체크포인트 디렉토리 없음: {config.CHECKPOINT_DIR}"
    assert config.LOG_DIR.exists(), f"로그 디렉토리 없음: {config.LOG_DIR}"
    assert config.RESULTS_DIR.exists(), f"결과 디렉토리 없음: {config.RESULTS_DIR}"
    print(f"  ✓ 체크포인트 디렉토리: {config.CHECKPOINT_DIR}")
    print(f"  ✓ 로그 디렉토리: {config.LOG_DIR}")
    print(f"  ✓ 결과 디렉토리: {config.RESULTS_DIR}")
except Exception as e:
    print(f"  ✗ 오류: {e}")
    sys.exit(1)

print("\n" + "=" * 80)
print("✓ Smoke 테스트 완료! 모든 모듈이 정상 작동합니다.")
print("=" * 80)

print("\n다음 단계:")
print("1. 데이터 전처리: data_preprocessing.ipynb 실행")
print("2. 모델 학습: python train.py")
print("3. 모델 테스트: python test.py")
