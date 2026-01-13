"""
유틸리티 함수 - 메트릭 계산, 시각화 등
"""
import torch
import numpy as np
from sklearn.metrics import accuracy_score, precision_recall_fscore_support, confusion_matrix, classification_report
import matplotlib.pyplot as plt
try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False
    print("경고: seaborn이 설치되지 않았습니다. matplotlib로 대체합니다.")
from pathlib import Path

def calculate_metrics(y_true, y_pred, class_names):
    """성능 지표 계산"""
    
    # 전체 정확도
    accuracy = accuracy_score(y_true, y_pred)
    
    # 클래스별 precision, recall, f1-score
    precision, recall, f1, support = precision_recall_fscore_support(
        y_true, y_pred, average=None, labels=range(len(class_names)), zero_division=0
    )
    
    # 평균 메트릭
    avg_precision, avg_recall, avg_f1, _ = precision_recall_fscore_support(
        y_true, y_pred, average='weighted', zero_division=0
    )
    
    # Confusion Matrix
    cm = confusion_matrix(y_true, y_pred, labels=range(len(class_names)))
    
    # Classification Report
    report = classification_report(
        y_true, 
        y_pred, 
        target_names=class_names,
        labels=range(len(class_names)),  # 모든 클래스 명시
        zero_division=0  # 예측 없는 클래스는 0으로 처리
    )
    
    metrics = {
        'accuracy': accuracy,
        'precision': precision,
        'recall': recall,
        'f1_score': f1,
        'support': support,
        'avg_precision': avg_precision,
        'avg_recall': avg_recall,
        'avg_f1': avg_f1,
        'confusion_matrix': cm,
        'classification_report': report
    }
    
    return metrics


def save_metrics_to_file(metrics, class_names, save_path):
    """메트릭을 txt 파일로 저장"""
    
    with open(save_path, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("행동 인식 모델 테스트 결과\n")
        f.write("=" * 80 + "\n\n")
        
        # 전체 정확도
        f.write(f"전체 정확도 (Overall Accuracy): {metrics['accuracy']:.4f}\n\n")
        
        # 평균 메트릭
        f.write("평균 메트릭 (Weighted Average):\n")
        f.write(f"  Precision: {metrics['avg_precision']:.4f}\n")
        f.write(f"  Recall:    {metrics['avg_recall']:.4f}\n")
        f.write(f"  F1-Score:  {metrics['avg_f1']:.4f}\n\n")
        
        # 클래스별 메트릭
        f.write("클래스별 성능 지표:\n")
        f.write("-" * 80 + "\n")
        f.write(f"{'클래스':<15} {'Precision':<12} {'Recall':<12} {'F1-Score':<12} {'Support':<10}\n")
        f.write("-" * 80 + "\n")
        
        for i, class_name in enumerate(class_names):
            f.write(f"{class_name:<15} "
                   f"{metrics['precision'][i]:<12.4f} "
                   f"{metrics['recall'][i]:<12.4f} "
                   f"{metrics['f1_score'][i]:<12.4f} "
                   f"{int(metrics['support'][i]):<10}\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("Confusion Matrix:\n")
        f.write("=" * 80 + "\n")
        
        # Confusion Matrix 출력
        cm = metrics['confusion_matrix']
        
        # 헤더
        f.write(f"{'실제\\예측':<15}")
        for class_name in class_names:
            f.write(f"{class_name:<12}")
        f.write("\n" + "-" * 80 + "\n")
        
        # 각 행
        for i, class_name in enumerate(class_names):
            f.write(f"{class_name:<15}")
            for j in range(len(class_names)):
                f.write(f"{int(cm[i, j]):<12}")
            f.write("\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("상세 분류 보고서:\n")
        f.write("=" * 80 + "\n")
        f.write(metrics['classification_report'])
        f.write("\n")
    
    print(f"✓ 결과 저장 완료: {save_path}")


def plot_confusion_matrix(cm, class_names, save_path):
    """Confusion Matrix 시각화"""
    
    plt.figure(figsize=(10, 8))
    
    if HAS_SEABORN:
        # seaborn 사용
        import seaborn as sns
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                    xticklabels=class_names, yticklabels=class_names)
    else:
        # matplotlib만 사용
        plt.imshow(cm, cmap='Blues', aspect='auto')
        plt.colorbar()
        
        # 축 레이블 설정
        plt.xticks(range(len(class_names)), class_names, rotation=45, ha='right')
        plt.yticks(range(len(class_names)), class_names)
        
        # 각 셀에 값 표시
        for i in range(len(class_names)):
            for j in range(len(class_names)):
                plt.text(j, i, str(int(cm[i, j])),
                        ha="center", va="center",
                        color="white" if cm[i, j] > cm.max() / 2 else "black")
    
    plt.title('Confusion Matrix', fontsize=16, fontweight='bold')
    plt.ylabel('실제 (True Label)', fontsize=12)
    plt.xlabel('예측 (Predicted Label)', fontsize=12)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"✓ Confusion Matrix 저장: {save_path}")


def plot_training_history(history, save_path):
    """학습 곡선 시각화"""
    
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    
    # Loss 곡선
    axes[0].plot(history['train_loss'], label='Train Loss', marker='o')
    axes[0].plot(history['val_loss'], label='Validation Loss', marker='s')
    axes[0].set_xlabel('Epoch')
    axes[0].set_ylabel('Loss')
    axes[0].set_title('Training and Validation Loss')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Accuracy 곡선
    axes[1].plot(history['train_acc'], label='Train Accuracy', marker='o')
    axes[1].plot(history['val_acc'], label='Validation Accuracy', marker='s')
    axes[1].set_xlabel('Epoch')
    axes[1].set_ylabel('Accuracy')
    axes[1].set_title('Training and Validation Accuracy')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"✓ 학습 곡선 저장: {save_path}")


class AverageMeter:
    """평균 계산 헬퍼 클래스"""
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0
    
    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count
