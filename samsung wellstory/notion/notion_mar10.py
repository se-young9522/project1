import json, urllib.request, sys
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'

headers = {
    'Authorization': f'Bearer {API_KEY}',
    'Notion-Version': '2022-06-28',
    'Content-Type': 'application/json',
}

def api(url, data=None, method='GET'):
    req = urllib.request.Request(
        url,
        data=json.dumps(data).encode('utf-8') if data else None,
        headers=headers,
        method=method,
    )
    try:
        resp = urllib.request.urlopen(req)
        return json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        body = e.read().decode('utf-8')
        print(f'ERROR {e.code}: {body[:500]}')
        raise

# ====== Samsung Wellstory 프로젝트 페이지 ID 찾기 ======
print('Samsung Wellstory 프로젝트 페이지 찾는 중...')
result = api('https://api.notion.com/v1/search', {
    'query': 'Samsung Wellstory',
    'filter': {'property': 'object', 'value': 'page'},
}, method='POST')

sw_page_id = None
for page in result.get('results', []):
    props = page.get('properties', {})
    title_arr = props.get('title', {}).get('title', [])
    if title_arr:
        title = title_arr[0].get('plain_text', '')
        # Samsung Wellstory 프로젝트 (프로젝트 진행 일지 하위)
        parent = page.get('parent', {})
        if title == 'Samsung Wellstory' and parent.get('type') == 'page_id':
            sw_page_id = page['id']
            print(f'  찾음: {sw_page_id}')
            break

if not sw_page_id:
    print('Samsung Wellstory 프로젝트 페이지를 찾지 못했습니다.')
    sys.exit(1)

# ====== 3월 10일 페이지 생성 ======
print('2026-03-10 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-10 (월)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']
print(f'  ID: {day_page_id}')

# ====== 작업 1: 데이터 수집 (프레임 추출) ======
print('작업 1: 데이터 수집...')

code_frame_test = """import cv2, os

video_path = r'C:\\Users\\ASUS\\Desktop\\bowl_video.mp4'
output_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'

cap = cv2.VideoCapture(video_path)
start_index = 1500  # 기존 이미지 이후부터
count, total, interval = 0, 500, 3  # 500장, 3프레임마다 1장

frame_num = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret or count >= total:
        break
    if frame_num % interval == 0:  # 매 3프레임마다 저장
        file_name = os.path.join(output_folder, f"bowl_{start_index+count:05d}.jpg")
        cv2.imwrite(file_name, frame)
        count += 1
    frame_num += 1
cap.release()"""

code_frame_side = """import cv2, os

video_path = r'C:\\Users\\ASUS\\Desktop\\bowl_video.mp4'
output_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'

# 측면 촬영 구간 (초 단위)
side_sections = [(45,58), (69,75), (88,99)]

cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
# 기존 이미지 번호 이어서 저장
existing = [f for f in os.listdir(output_folder)
            if f.startswith('bowl_') and f.endswith('.jpg')]
start_index = max([int(f[5:10]) for f in existing], default=-1)+1

saved, interval, max_save = 0, 3, 300
for (start_sec, end_sec) in side_sections:
    if saved >= max_save: break
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(start_sec*fps))
    for i in range(int((end_sec-start_sec)*fps)):
        if saved >= max_save: break
        ret, frame = cap.read()
        if not ret: break
        if i % interval == 0:  # 3프레임마다 저장
            cv2.imwrite(os.path.join(output_folder,
                f"bowl_{start_index+saved:05d}.jpg"), frame)
            saved += 1
cap.release()"""

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. 데이터 수집 (프레임 추출)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '영상에서 공정 뷰 + 측면 뷰 프레임 추출'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '총 ~3000장 수집 (공정 2300장 + 측면 700장)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: frame_test.py - 공정 뷰 프레임 추출'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '영상에서 매 3프레임마다 1장씩 추출하여 공정(위에서 내려다보는) 뷰 이미지 500장 저장'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_frame_test}}],
            'language': 'python'}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: frame_side.py - 측면 뷰 프레임 추출'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '영상의 특정 시간 구간(측면 촬영)에서 프레임 추출. 기존 이미지 번호를 이어서 저장'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_frame_side}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ====== 작업 2: COCO 기반 자동 라벨링 (bbox) ======
print('작업 2: 자동 라벨링...')

code_auto_label = """from ultralytics import YOLO
import os

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'

model = YOLO('yolov8x.pt')  # COCO 사전학습 모델
BOWL_CLASS_COCO = 45  # COCO에서 bowl = 45번

for img_file in os.listdir(image_folder):
    if not img_file.endswith('.jpg'): continue
    results = model(os.path.join(image_folder, img_file),
                    verbose=False)
    label_lines = []
    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls == BOWL_CLASS_COCO and conf >= 0.5:
                # YOLO bbox: cx cy w h (0~1 정규화)
                cx, cy, w, h = box.xywhn[0].tolist()
                label_lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    if label_lines:
        with open(os.path.join(label_folder,
                  img_file.replace('.jpg','.txt')), 'w') as f:
            f.write('\\n'.join(label_lines))"""

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. COCO 기반 자동 라벨링 (bbox)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'COCO 사전학습 yolov8x 모델로 bowl 클래스 자동 bbox 라벨링'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'conf >= 0.5 필터링, YOLO format(cx, cy, w, h) 저장'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: auto_label.py - COCO 모델로 bbox 자동 라벨링'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'yolov8x(COCO) 모델로 bowl 클래스(45번)를 감지하여 YOLO bbox 형식 txt 라벨 자동 생성'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_auto_label}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ====== 작업 3: 1차 detection 학습 ======
print('작업 3: detection 학습...')

code_train_v1 = """import yaml, os, shutil, random
from ultralytics import YOLO

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'
base_dir = r'C:\\Users\\ASUS\\Desktop\\datasets'

# 라벨 있는 이미지만 사용 -> 80/20 분리
labeled = [f for f in os.listdir(image_folder)
    if f.endswith('.jpg') and
    os.path.exists(os.path.join(label_folder, f.replace('.jpg','.txt')))]
random.shuffle(labeled)
split = int(len(labeled) * 0.8)

# train/val 폴더에 복사 후 YAML 생성
cfg = {'path': base_dir, 'train': 'images/train',
       'val': 'images/val', 'names': {0: 'bowl'}}

# yolov8s detection 모델 학습
model = YOLO('yolov8s.pt')  # detection용 (seg 아님)
model.train(
    data='bowl_v1.yaml', epochs=100, imgsz=640,
    batch=16, name='bowl_v2', patience=20,
    device=0, workers=0,
    fliplr=0.5, degrees=15, scale=0.3)"""

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 1차 Detection 모델 학습'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'yolov8s.pt 기반 detection 학습 (bbox)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'epochs=100, patience=20, 데이터 증강 적용'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: train_v1.py - YOLOv8s detection 학습'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'bbox 라벨 기반 YOLOv8s detection 모델 학습. train/val 80/20 분리 후 YAML 생성하여 학습 실행'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_train_v1}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ====== 작업 4: semi auto label ======
print('작업 4: semi auto label...')

code_semi = """from ultralytics import YOLO
import os

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'
model_path = r'C:\\Users\\ASUS\\runs\\detect\\bowl_v14\\weights\\best.pt'

model = YOLO(model_path)  # 1차 학습된 모델 로드

# 라벨 없는 이미지만 대상
unlabeled = [f for f in os.listdir(image_folder)
    if f.endswith('.jpg') and
    not os.path.exists(os.path.join(label_folder, f.replace('.jpg','.txt')))]

for img_file in unlabeled:
    results = model(os.path.join(image_folder, img_file),
                    verbose=False, conf=0.35, device='cpu')
    label_lines = []
    for r in results:
        for box in r.boxes:
            cx, cy, w, h = box.xywhn[0].tolist()
            label_lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    if label_lines:
        with open(os.path.join(label_folder,
                  img_file.replace('.jpg','.txt')), 'w') as f:
            f.write('\\n'.join(label_lines))"""

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. Semi-Auto 라벨링 (학습 모델 활용)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '1차 학습된 모델(bowl_v14)로 미라벨 이미지 자동 라벨링'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'conf >= 0.35 기준, 라벨 없는 이미지만 대상'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: semi_auto_label.py - 학습 모델로 재라벨링'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '1차 학습된 detection 모델(bowl_v14)을 활용하여 아직 라벨이 없는 이미지에 bbox 자동 생성'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_semi}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ====== 작업 5: RealSense 관련 ======
print('작업 5: RealSense...')

code_capture = """import pyrealsense2 as rs
import numpy as np, cv2, os

output_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
os.makedirs(output_folder, exist_ok=True)

# 기존 이미지 번호 이어서 저장
existing = [f for f in os.listdir(output_folder)
            if f.startswith('bowl_') and f.endswith('.jpg')]
start_index = max([int(f[5:10]) for f in existing], default=-1)+1

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

saved = 0
while True:
    frames = pipeline.wait_for_frames()
    img = np.asanyarray(frames.get_color_frame().get_data())
    cv2.imshow('RealSense Capture', img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):     # 스페이스바: 저장
        cv2.imwrite(os.path.join(output_folder,
            f"bowl_{start_index+saved:05d}.jpg"), img)
        saved += 1
    elif key == ord('q'):   # q: 종료
        break
pipeline.stop()"""

code_detect = """import pyrealsense2 as rs
import numpy as np, cv2
from ultralytics import YOLO

model = YOLO(r'C:\\Users\\ASUS\\runs\\detect\\bowl_v22\\weights\\best.pt')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)  # depth를 color에 정렬

while True:
    frames = align.process(pipeline.wait_for_frames())
    color = np.asanyarray(frames.get_color_frame().get_data())
    depth = frames.get_depth_frame()
    results = model(color, conf=0.5, verbose=False)
    for r in results:
        for box in r.boxes:
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            cx, cy = (x1+x2)//2, (y1+y2)//2
            dist = depth.get_distance(cx, cy)  # 중심점 거리(m)
            cv2.rectangle(color, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(color, f"Bowl {float(box.conf[0]):.0%} {dist:.2f}m",
                (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    cv2.imshow('Detection', color)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
pipeline.stop()"""

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '5. RealSense 카메라 연동'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealSense로 실시간 데이터 수집 + 추론 파이프라인 구축'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'depth 정렬로 bowl 중심점 거리 측정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: realsense_capture.py - 실시간 이미지 수집'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealSense 카메라로 실시간 미리보기하며 스페이스바로 이미지 수동 캡처. 기존 번호 이어서 저장'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_capture}}],
            'language': 'python'}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드: realsense_detect.py - 실시간 추론 + 거리 측정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '학습된 모델(bowl_v22)로 실시간 bowl 감지 + depth 센서로 중심점 거리(m) 측정하여 화면에 표시'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_detect}}],
            'language': 'python'}},
    ]
}, method='PATCH')

print('\n완료!')
print('2026-03-10 (월)')
print('  1. 데이터 수집 (frame_test.py + frame_side.py)')
print('  2. COCO 자동 라벨링 (auto_label.py)')
print('  3. 1차 Detection 학습 (train_v1.py)')
print('  4. Semi-Auto 라벨링 (semi_auto_label.py)')
print('  5. RealSense 연동 (realsense_capture.py + realsense_detect.py)')
