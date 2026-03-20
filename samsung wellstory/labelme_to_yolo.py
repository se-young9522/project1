"""
labelme JSON → YOLO polygon txt 변환 스크립트
polygon + rectangle 라벨 모두 지원
라벨링 완료 후 이 파일을 실행하세요
"""
import json
import os

image_folder = r'C:\Users\ASUS\Desktop\datasets\videos\images'
label_folder = r'C:\Users\ASUS\Desktop\datasets\videos\labels'
class_name   = 'bowl'  # labelme에서 입력한 클래스 이름과 동일해야 함

os.makedirs(label_folder, exist_ok=True)

json_files = [f for f in os.listdir(image_folder) if f.endswith('.json')]
print(f"변환할 JSON 파일: {len(json_files)}개\n")

converted = 0
skipped   = 0

for json_file in json_files:
    json_path = os.path.join(image_folder, json_file)

    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    img_w = data['imageWidth']
    img_h = data['imageHeight']

    label_lines = []
    for shape in data['shapes']:
        if shape['label'] != class_name:
            continue

        pts = shape['points']

        if shape['shape_type'] == 'polygon':
            # polygon → YOLO seg 포맷 (정규화된 꼭짓점 좌표)
            normalized = []
            for px, py in pts:
                normalized.append(f"{px / img_w:.6f}")
                normalized.append(f"{py / img_h:.6f}")
            label_lines.append(f"0 {' '.join(normalized)}")

        elif shape['shape_type'] == 'rectangle':
            # rectangle → 4꼭짓점 polygon으로 변환
            x1, y1 = pts[0]
            x2, y2 = pts[1]
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)

            label_lines.append(
                f"0 {x1/img_w:.6f} {y1/img_h:.6f} "
                f"{x2/img_w:.6f} {y1/img_h:.6f} "
                f"{x2/img_w:.6f} {y2/img_h:.6f} "
                f"{x1/img_w:.6f} {y2/img_h:.6f}"
            )

    if label_lines:
        txt_name = json_file.replace('.json', '.txt')
        txt_path = os.path.join(label_folder, txt_name)
        with open(txt_path, 'w') as f:
            f.write('\n'.join(label_lines))
        converted += 1
    else:
        skipped += 1

print(f"변환 완료: {converted}개")
print(f"스킵 (bowl 라벨 없음): {skipped}개")
print(f"저장 위치: {label_folder}")
