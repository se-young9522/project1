"""
segmentation 재학습 스크립트 (전체 ~2870장)
"""
import yaml, os, shutil, random
from ultralytics import YOLO

# ─── 경로 설정 ─────────────────────────────────────────
image_folder = r'C:\Users\ASUS\Desktop\datasets\videos\images'
label_folder = r'C:\Users\ASUS\Desktop\datasets\videos\labels'
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

# 이전 train/val 초기화 후 재생성
for split_name in ['train', 'val']:
    for sub in ['images', 'labels']:
        d = os.path.join(base_dir, sub, split_name)
        if os.path.exists(d):
            shutil.rmtree(d)
        os.makedirs(d, exist_ok=True)

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
yaml_path = r'C:\Users\ASUS\Desktop\bowl_seg.yaml'
with open(yaml_path, 'w') as f:
    yaml.dump(cfg, f, default_flow_style=False)

# seg 모델 학습 (yolov8s-seg)
model = YOLO('yolov8s-seg.pt')
model.train(
    data=yaml_path,
    epochs=150,
    imgsz=640,
    batch=16,
    name='bowl_seg_v3',
    patience=30,
    device=0,
    workers=0,
    fliplr=0.5,
    degrees=20,
    scale=0.4,
    mosaic=0.5,
    mixup=0.05,
    hsv_h=0.02,
    hsv_s=0.6,
    hsv_v=0.4,
    translate=0.15,
    copy_paste=0.15,
    perspective=0.0003,
)

print("\nseg 재학습 완료!")
print("모델 위치: runs/segment/bowl_seg_v3/weights/best.pt")
