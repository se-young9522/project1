import yaml, os, shutil, random
from ultralytics import YOLO

# ─── 경로 설정 ─────────────────────────────────────────
image_folder = r'C:\Users\ASUS\Desktop\datasets\images'
label_folder = r'C:\Users\ASUS\Desktop\datasets\labels'
base_dir     = r'C:\Users\ASUS\Desktop\datasets'
# ────────────────────────────────────────────────────────

# 라벨이 있는 이미지만 사용
labeled = [
    f for f in os.listdir(image_folder)
    if f.endswith('.jpg') and
       os.path.exists(os.path.join(label_folder, f.replace('.jpg', '.txt')))
]
print(f"라벨된 이미지: {len(labeled)}장")

# train/val 분리 (80/20)
random.shuffle(labeled)
split = int(len(labeled) * 0.8)
train_files = labeled[:split]
val_files   = labeled[split:]

for split_name in ['train', 'val']:
    os.makedirs(os.path.join(base_dir, 'images', split_name), exist_ok=True)
    os.makedirs(os.path.join(base_dir, 'labels', split_name), exist_ok=True)

def copy_files(file_list, split_name):
    for fname in file_list:
        label = fname.replace('.jpg', '.txt')
        shutil.copy(os.path.join(image_folder, fname),
                    os.path.join(base_dir, 'images', split_name, fname))
        shutil.copy(os.path.join(label_folder, label),
                    os.path.join(base_dir, 'labels', split_name, label))

copy_files(train_files, 'train')
copy_files(val_files,   'val')
print(f"train: {len(train_files)}장 / val: {len(val_files)}장")

# YAML 생성
cfg = {
    'path':  base_dir,
    'train': 'images/train',
    'val':   'images/val',
    'names': {0: 'bowl'}
}
yaml_path = r'C:\Users\ASUS\Desktop\bowl_v1.yaml'
with open(yaml_path, 'w') as f:
    yaml.dump(cfg, f, default_flow_style=False)

# 1차 학습
model = YOLO('yolov8s.pt')
model.train(
    data=yaml_path,
    epochs=100,
    imgsz=640,
    batch=16,
    name='bowl_v2',
    patience=20,
    device=0,
    workers=0,
    fliplr=0.5,   # 좌우 반전 증강
    degrees=15,   # 회전 증강 (±15도)
    scale=0.3,    # 크기 변화 증강
)

print("\n1차 학습 완료!")
print("모델 위치: runs/detect/bowl_v1/weights/best.pt")
