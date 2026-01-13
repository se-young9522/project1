"""
모델 정의 - ResNet18 기반 행동 분류 모델
"""
import torch
import torch.nn as nn
from torchvision import models

class ActionRecognitionModel(nn.Module):
    """행동 인식 CNN 모델"""
    
    def __init__(self, num_classes=6, pretrained=True):
        """
        Args:
            num_classes: 출력 클래스 수
            pretrained: ImageNet 사전 학습 가중치 사용 여부
        """
        super(ActionRecognitionModel, self).__init__()
        
        # ResNet18 백본
        self.backbone = models.resnet18(pretrained=pretrained)
        
        # 마지막 FC 레이어를 우리 클래스 수에 맞게 교체
        num_features = self.backbone.fc.in_features
        self.backbone.fc = nn.Linear(num_features, num_classes)
        
    def forward(self, x):
        return self.backbone(x)


def get_model(num_classes=6, pretrained=True, device='cuda'):
    """모델 생성 및 디바이스로 이동"""
    model = ActionRecognitionModel(num_classes=num_classes, pretrained=pretrained)
    model = model.to(device)
    return model


if __name__ == '__main__':
    # 모델 테스트
    model = get_model(num_classes=6, device='cpu')
    print(model)
    
    # 더미 입력으로 테스트
    dummy_input = torch.randn(2, 3, 224, 224)
    output = model(dummy_input)
    print(f"\n입력 shape: {dummy_input.shape}")
    print(f"출력 shape: {output.shape}")
