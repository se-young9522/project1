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

# Samsung Wellstory 프로젝트 페이지 찾기
print('Samsung Wellstory 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{PROJECT_PAGE_ID}/children?page_size=100')
sw_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        print(f'  발견: {title}')
        if 'Samsung' in title or 'Wellstory' in title or '삼성' in title:
            sw_page_id = block['id']
            break

if not sw_page_id:
    print('Samsung Wellstory 페이지를 찾을 수 없습니다.')
    sys.exit(1)

print(f'Samsung Wellstory 페이지 ID: {sw_page_id}')

# 3월 11일 날짜 서브페이지 생성
print('2026-03-11 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-11 (화)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']

# 코드 준비
code_filter = """import os
import numpy as np

label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'

def polygon_area(coords):
    # Shoelace formula: 다각형 꼭짓점 좌표로 면적 계산
    xs = coords[0::2]  # x좌표만 추출 (짝수 인덱스)
    ys = coords[1::2]  # y좌표만 추출 (홀수 인덱스)
    n = len(xs)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        j = (i + 1) % n  # 다음 꼭짓점 (마지막→첫번째)
        area += xs[i] * ys[j]
        area -= xs[j] * ys[i]
    return abs(area) / 2.0

label_files = [f for f in os.listdir(label_folder)
               if f.endswith('.txt')]

for fname in label_files:
    fpath = os.path.join(label_folder, fname)
    with open(fpath, 'r') as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]
    if len(lines) <= 1:  # polygon 1개면 스킵
        continue
    # 각 polygon의 면적 비교 → 가장 큰 것만 유지
    best_line, best_area = lines[0], 0.0
    for line in lines:
        parts = line.split()
        coords = list(map(float, parts[1:]))  # class_id 제외
        area = polygon_area(coords)
        if area > best_area:
            best_area = area
            best_line = line
    with open(fpath, 'w') as f:  # 최대 polygon만 저장
        f.write(best_line + '\\n')"""

code_remove = """import os

LABEL_DIR = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'

# verify_labels.py 뷰어에서 확인한 오탐 인덱스 (1-based)
bad_indices = []
# 개별 인덱스
for i in [1351, 1353, 1407, 1410, 1786, ...]:
    bad_indices.append(i)
# 연속 범위
for start, end in [(1354, 1358), (2078, 2084), ...]:
    bad_indices.extend(range(start, end + 1))

# 뷰어와 동일한 sorted() 순서로 파일 목록 생성
label_files = sorted(f for f in os.listdir(LABEL_DIR)
                     if f.endswith('.txt'))

for idx in sorted(set(bad_indices)):
    i = idx - 1  # 1-based → 0-based 변환
    if 0 <= i < len(label_files):
        fpath = os.path.join(LABEL_DIR, label_files[i])
        os.remove(fpath)  # 오탐 라벨 파일 삭제"""

code_train = """import yaml, os, shutil, random
from ultralytics import YOLO

image_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\images'
label_folder = r'C:\\Users\\ASUS\\Desktop\\datasets\\labels'
base_dir = r'C:\\Users\\ASUS\\Desktop\\datasets'

# 라벨 있는 이미지만 필터링
labeled = [f for f in os.listdir(image_folder)
    if f.endswith('.jpg') and
    os.path.exists(os.path.join(label_folder,
                   f.replace('.jpg','.txt')))]

random.shuffle(labeled)
split = int(len(labeled) * 0.8)
train_files, val_files = labeled[:split], labeled[split:]

# 이전 데이터 초기화 후 재생성
for s in ['train', 'val']:
    for sub in ['images', 'labels']:
        d = os.path.join(base_dir, sub, s)
        if os.path.exists(d):
            shutil.rmtree(d)  # 이전 데이터 삭제
        os.makedirs(d, exist_ok=True)"""

code_train_2 = """# YAML 설정
cfg = {'path': base_dir, 'train': 'images/train',
       'val': 'images/val', 'names': {0: 'bowl'}}

model = YOLO('yolov8s-seg.pt')
model.train(
    data=yaml_path,
    epochs=150,      # 100→150 (데이터 20배 증가)
    imgsz=640,
    batch=16,        # 32시도→VRAM부족→16복원
    name='bowl_seg_v3',
    patience=30,     # 20→30 (수렴 여유)
    device=0,
    workers=0,
    fliplr=0.5,      # 좌우반전 50%
    degrees=15,      # 회전 +-15도
    scale=0.3,       # 크기변환 +-30%
    mosaic=0.5,      # 4장 합성 (다양한 구도)
    hsv_h=0.015,     # 색조 변화
    hsv_s=0.5,       # 채도 변화
    hsv_v=0.3,       # 명도 변화 (조명 대응)
    translate=0.1,   # 위치 이동
    copy_paste=0.1,  # seg전용: 마스크 복붙
)"""

print('내용 작성 중...')

# 작업 1: filter_labels.py
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. 자동 라벨 필터링 (filter_labels.py)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '자동 라벨링 결과 중 다중 polygon이 있는 파일 917개 발견'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Shoelace formula로 면적 계산 → 가장 큰 polygon만 유지'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '그릇 외 영역(노트북 화면 등) 오탐 제거'}}]}},
    ]
}, method='PATCH')

# 코드리뷰: filter_labels.py
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: filter_labels.py - 다중 polygon에서 최대 면적만 유지하는 스크립트'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_filter}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 작업 2: 오탐 삭제
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. 라벨 검수 및 오탐 삭제 (remove_bad_labels.py)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'verify_labels.py로 전체 라벨 시각적 검수'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '비정상 라벨 61개 식별 후 삭제 (polygon 선 이탈, 노트북 화면 오탐)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '최종 유효 라벨: ~2869장'}}]}},
    ]
}, method='PATCH')

# 코드리뷰: remove_bad_labels.py
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: remove_bad_labels.py - 뷰어에서 확인한 오탐 라벨 삭제 스크립트'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_remove}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 작업 3: train_seg.py 수정
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 재학습 스크립트 수정 (train_seg.py)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'epochs 100→150, patience 20→30으로 조정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'augmentation 추가 (mosaic, hsv, translate, copy_paste)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'batch 32 시도 → VRAM 8GB 풀사용으로 느려서 batch 16 복원'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'train/val 폴더 초기화 로직 추가 (이전 데이터 혼입 방지)'}}]}},
    ]
}, method='PATCH')

# 코드리뷰: train_seg.py
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: train_seg.py - ~2869장 전체 재학습 스크립트'}}]}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_train}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'code', 'code': {
            'rich_text': [{'type': 'text', 'text': {'content': code_train_2}}],
            'language': 'python'}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

print('\n완료!')
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       ├── 2026-03-11 (화) ← 기존')
print('       └── 2026-03-11 (화) ← 신규 (오늘 작업)')
