# 행동 인식 모델 학습 프로젝트

PyTorch를 사용한 KTH 행동 인식 데이터셋 학습 프로젝트입니다.

## 프로젝트 구조

```
archive/
├── data_preprocess/          # 전처리된 데이터 (data_preprocessing.ipynb 실행 후 생성)
│   ├── train/
│   ├── validation/
│   └── test/
├── checkpoints/              # 학습된 모델 체크포인트
├── logs/                     # 학습 로그
├── results/                  # 테스트 결과 및 시각화
├── config.py                 # 설정 파일
├── dataset.py                # 데이터셋 클래스
├── model.py                  # 모델 정의 (ResNet18)
├── utils.py                  # 유틸리티 함수
├── train.py                  # 학습 스크립트
├── test.py                   # 테스트 스크립트
├── smoke_test.py             # 모듈 테스트
└── data_preprocessing.ipynb  # 데이터 전처리 노트북
```

## 필요한 패키지

```bash
pip install torch torchvision opencv-python pandas matplotlib seaborn scikit-learn tqdm pillow
```

또는 conda 환경에서:

```bash
conda install pytorch torchvision -c pytorch
pip install opencv-python pandas matplotlib seaborn scikit-learn tqdm pillow
```

## 사용 방법

### 1단계: 데이터 전처리

먼저 `data_preprocessing.ipynb` 노트북을 실행하여 데이터를 전처리합니다:

1. Jupyter Notebook 실행
2. `data_preprocessing.ipynb` 열기
3. 모든 셀 순서대로 실행
4. `data_preprocess/` 폴더에 train/validation/test 데이터 생성 확인

### 2단계: Smoke 테스트 (선택사항)

모든 모듈이 정상 작동하는지 확인:

```bash
python smoke_test.py
```

### 3단계: 모델 학습

```bash
python train.py
```

**학습 과정:**
- ResNet18 백본 사용 (ImageNet 사전 학습)
- 6개 행동 클래스 분류: boxing, handclapping, handwaving, jogging, running, walking
- 자동으로 최고 성능 모델 저장 (`checkpoints/best_model.pth`)
- 학습 곡선 시각화 (`results/training_history.png`)
- Early stopping 지원 (10 epoch 동안 개선 없으면 중단)

**주요 하이퍼파라미터 (config.py에서 수정 가능):**
- Batch Size: 16
- Epochs: 30
- Learning Rate: 0.001
- Optimizer: Adam
- Scheduler: ReduceLROnPlateau

### 4단계: 모델 테스트

```bash
python test.py
```

**테스트 결과:**
- 성능 지표 계산 (Accuracy, Precision, Recall, F1-Score)
- Confusion Matrix 생성
- 결과를 `results/test_results.txt`에 저장
- Confusion Matrix를 `results/confusion_matrix.png`에 저장

## 출력 파일

### 학습 후
- `checkpoints/best_model.pth`: 최고 성능 모델
- `checkpoints/last_model.pth`: 마지막 epoch 모델
- `results/training_history.png`: 학습/검증 곡선

### 테스트 후
- `results/test_results.txt`: 상세 성능 지표
- `results/confusion_matrix.png`: Confusion Matrix 시각화

## 성능 지표

`test_results.txt`에 다음 정보가 포함됩니다:

1. **전체 정확도 (Overall Accuracy)**
2. **평균 메트릭 (Weighted Average)**
   - Precision
   - Recall
   - F1-Score
3. **클래스별 성능**
   - 각 행동 클래스의 Precision, Recall, F1-Score
4. **Confusion Matrix**
5. **상세 분류 보고서**

## 모델 아키텍처

- **백본**: ResNet18 (ImageNet 사전 학습)
- **입력**: 224x224 RGB 이미지
- **출력**: 6개 클래스 확률
- **데이터 증강**:
  - Random Horizontal Flip
  - Random Rotation (±10도)
  - Color Jitter
  - ImageNet Normalization

## 문제 해결

### 메모리 부족 오류
`config.py`에서 `BATCH_SIZE`를 줄이세요 (예: 8 또는 4).

### CUDA 오류
GPU가 없는 경우 `config.py`에서 `DEVICE = 'cpu'`로 설정하세요.

### 데이터 로딩 오류
Windows에서는 `config.py`의 `NUM_WORKERS = 0`으로 유지하세요.

## 커스터마이징

### 다른 모델 사용
`model.py`에서 ResNet18 대신 다른 모델 사용 가능:
- ResNet34, ResNet50
- MobileNetV2
- EfficientNet

### 하이퍼파라미터 조정
`config.py`에서 모든 하이퍼파라미터 수정 가능.

## 라이선스

이 프로젝트는 교육 목적으로 작성되었습니다.
