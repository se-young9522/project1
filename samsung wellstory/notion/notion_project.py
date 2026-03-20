import json, urllib.request, sys
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
PROJECT_PAGE_ID = '32015435-1a92-813d-a3f0-e433e57bc50a'

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

# Step 0: 기존 블록 삭제
print('기존 블록 삭제 중...')
blocks = api(f'https://api.notion.com/v1/blocks/{PROJECT_PAGE_ID}/children?page_size=100')
for block in blocks.get('results', []):
    try:
        api(f'https://api.notion.com/v1/blocks/{block["id"]}', method='DELETE')
    except:
        pass

# Step 1: 안내 문구
print('안내 문구 추가...')
api(f'https://api.notion.com/v1/blocks/{PROJECT_PAGE_ID}/children', {
    'children': [
        {
            'object': 'block',
            'type': 'callout',
            'callout': {
                'rich_text': [{'type': 'text', 'text': {'content': '각 프로젝트를 클릭하면 날짜별 상세 진행 내용을 확인할 수 있습니다.'}}],
                'icon': {'type': 'emoji', 'emoji': '\U0001f4a1'},
            },
        },
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# Step 2: Samsung Wellstory 프로젝트 서브페이지
print('Samsung Wellstory 프로젝트 페이지 생성...')
sw_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': PROJECT_PAGE_ID},
    'icon': {'type': 'emoji', 'emoji': '\U0001f916'},
    'properties': {
        'title': {'title': [{'text': {'content': 'Samsung Wellstory'}}]}
    },
}, method='POST')
sw_page_id = sw_page['id']

api(f'https://api.notion.com/v1/blocks/{sw_page_id}/children', {
    'children': [
        {
            'object': 'block',
            'type': 'callout',
            'callout': {
                'rich_text': [{'type': 'text', 'text': {'content': 'YOLOv8 + RealSense 카메라로 접시(bowl) 인식 후 로봇팔로 이동하는 프로젝트'}}],
                'icon': {'type': 'emoji', 'emoji': '\U0001f4cc'},
            },
        },
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# Step 3: 2026-03-11 날짜 서브페이지
print('2026-03-11 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-11 (화)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']

# Step 4: 오늘 내용
print('상세 내용 작성 중...')

code_labelme_1 = """import json  # JSON 파일 읽기
import os    # 파일경로 처리

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'  # 이미지+JSON 폴더
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'  # 변환결과 저장
class_name = 'bowl'  # 타겟 클래스명

os.makedirs(label_folder, exist_ok=True)  # 폴더 자동생성
json_files = [f for f in os.listdir(image_folder) if f.endswith('.json')]

for json_file in json_files:
    with open(os.path.join(image_folder, json_file), 'r', encoding='utf-8') as f:
        data = json.load(f)
    img_w = data['imageWidth']   # 정규화용 이미지 가로
    img_h = data['imageHeight']  # 정규화용 이미지 세로

    label_lines = []
    for shape in data['shapes']:
        if shape['label'] != class_name:  # bowl 아닌 라벨 무시
            continue
        pts = shape['points']"""

code_labelme_2 = """        if shape['shape_type'] == 'polygon':
            # polygon -> YOLO seg: "0 x1 y1 x2 y2..." (0~1 정규화)
            normalized = []
            for px, py in pts:
                normalized.append(f"{px/img_w:.6f}")  # x 정규화
                normalized.append(f"{py/img_h:.6f}")  # y 정규화
            label_lines.append(f"0 {' '.join(normalized)}")

        elif shape['shape_type'] == 'rectangle':
            # rectangle -> 4꼭짓점 (좌상->우상->우하->좌하)
            x1, y1 = pts[0]; x2, y2 = pts[1]
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            label_lines.append(
                f"0 {x1/img_w:.6f} {y1/img_h:.6f} "
                f"{x2/img_w:.6f} {y1/img_h:.6f} "
                f"{x2/img_w:.6f} {y2/img_h:.6f} "
                f"{x1/img_w:.6f} {y2/img_h:.6f}")

    if label_lines:  # bowl 라벨 있으면 저장
        txt_name = json_file.replace('.json', '.txt')
        with open(os.path.join(label_folder, txt_name), 'w') as f:
            f.write('\\n'.join(label_lines))"""

code_train_1 = """import yaml, os, shutil, random
from ultralytics import YOLO  # YOLOv8

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'
base_dir = r'C:\\Users\\ASUS\\Desktop\\datasets'

# 라벨 있는 이미지만 필터링
labeled = [f for f in os.listdir(image_folder)
    if f.endswith('.jpg') and
    os.path.exists(os.path.join(label_folder, f.replace('.jpg','.txt')))]

# 80/20 분리
random.shuffle(labeled)
split = int(len(labeled) * 0.8)
train_files, val_files = labeled[:split], labeled[split:]

for s in ['train', 'val']:
    os.makedirs(os.path.join(base_dir, 'images', s), exist_ok=True)
    os.makedirs(os.path.join(base_dir, 'labels', s), exist_ok=True)

def copy_files(file_list, split_name):
    for fname in file_list:
        label = fname.replace('.jpg', '.txt')
        shutil.copy(os.path.join(image_folder, fname),
                    os.path.join(base_dir, 'images', split_name, fname))
        shutil.copy(os.path.join(label_folder, label),
                    os.path.join(base_dir, 'labels', split_name, label))
copy_files(train_files, 'train')
copy_files(val_files, 'val')"""

code_train_2 = """# YAML 설정
cfg = {'path': base_dir, 'train': 'images/train',
       'val': 'images/val', 'names': {0: 'bowl'}}
with open(r'C:\\Users\\ASUS\\Desktop\\bowl_seg.yaml', 'w') as f:
    yaml.dump(cfg, f)

# YOLOv8s-seg 학습
model = YOLO('yolov8s-seg.pt')  # small 사전학습 모델
model.train(
    data=r'C:\\Users\\ASUS\\Desktop\\bowl_seg.yaml',
    epochs=100,      # 최대 100 에포크
    imgsz=640,       # 입력 640x640
    batch=16,        # 배치 사이즈
    name='bowl_seg_v1',
    patience=20,     # 20 에포크 개선없으면 Early Stop
    device=0,        # GPU 0번
    workers=0,       # Windows 오류 방지
    fliplr=0.5,      # 좌우반전 50%
    degrees=15,      # 회전 +-15도
    scale=0.3,       # 크기변환 +-30%
)"""

# Part 1: 작업 1
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. 수동 polygon 라벨링 완료'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '총 ~180장 라벨링 (공정 100장 + 측면 80장)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'labelme 도구로 polygon 형태 수동 라벨링'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# Part 2: 작업 2 설명
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. labelme_to_yolo.py 작성'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'labelme JSON -> YOLO polygon txt 변환'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'polygon + rectangle 라벨 모두 지원'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
    ]
}, method='PATCH')

# Part 3: labelme 코드 리뷰
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: labelme_to_yolo.py'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_labelme_1}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_labelme_2}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# Part 4: 작업 3 설명
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. train_seg.py 작성'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'YOLOv8s-seg 기반 segmentation 학습 스크립트'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'train/val 자동 분리 (80/20) + YAML 자동 생성'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
    ]
}, method='PATCH')

# Part 5: train 코드 리뷰
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: train_seg.py'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_train_1}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_train_2}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# Part 6: 작업 4
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. Notion API 연동'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Internal Integration 생성 + API 키 발급'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '일일 업무 보고 / 프로젝트 진행 일지 계층 구조 설계'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '워크스페이스 불필요 페이지 정리'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '(여기에 상세 내용을 작성하세요)'}, 'annotations': {'color': 'gray', 'italic': True}}]}},
    ]
}, method='PATCH')

print('완료!')
print()
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       └── 2026-03-11 (화)')
print('            ├── 1. 수동 polygon 라벨링 완료')
print('            ├── 2. labelme_to_yolo.py + 코드리뷰')
print('            ├── 3. train_seg.py + 코드리뷰')
print('            └── 4. Notion API 연동')
