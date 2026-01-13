"""
설정 파일 - 모델 학습 하이퍼파라미터 및 경로 설정
"""
from pathlib import Path

# 데이터 경로
BASE_DIR = Path(r'C:\Users\HKIT\Desktop\archive')
DATA_DIR = BASE_DIR / 'data_preprocess'

# 행동 클래스
CLASSES = ['boxing', 'handclapping', 'handwaving', 'jogging', 'running', 'walking']
NUM_CLASSES = len(CLASSES)

# 모델 설정
MODEL_NAME = 'resnet18'
INPUT_SIZE = 224
PRETRAINED = True

# 학습 하이퍼파라미터
BATCH_SIZE = 16  # 메모리 고려하여 작게 설정
NUM_EPOCHS = 30
LEARNING_RATE = 0.001
WEIGHT_DECAY = 1e-4

# 데이터 로더 설정
NUM_WORKERS = 0  # Windows에서는 0으로 설정
PIN_MEMORY = True

# 체크포인트 및 로그
CHECKPOINT_DIR = BASE_DIR / 'checkpoints'
LOG_DIR = BASE_DIR / 'logs'
RESULTS_DIR = BASE_DIR / 'results'

# 체크포인트 디렉토리 생성
CHECKPOINT_DIR.mkdir(exist_ok=True)
LOG_DIR.mkdir(exist_ok=True)
RESULTS_DIR.mkdir(exist_ok=True)

# 디바이스 설정
DEVICE = 'cuda'  # 'cuda' 또는 'cpu'

# Early Stopping
PATIENCE = 10  # validation loss가 개선되지 않으면 몇 epoch 후 중단

# 시드 설정 (재현성)
SEED = 42

print(f"설정 로드 완료")
print(f"데이터 디렉토리: {DATA_DIR}")
print(f"클래스 수: {NUM_CLASSES}")
print(f"배치 크기: {BATCH_SIZE}")
print(f"에포크: {NUM_EPOCHS}")
