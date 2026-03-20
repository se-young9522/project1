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
        if 'Samsung' in title or 'Wellstory' in title or '삼성' in title:
            sw_page_id = block['id']
            break

if not sw_page_id:
    print('Samsung Wellstory 페이지를 찾을 수 없습니다.')
    sys.exit(1)
print(f'Samsung Wellstory 페이지 ID: {sw_page_id}')

# ========================================
# 3월 12일 (수)
# ========================================
print('\n=== 2026-03-12 (수) 페이지 생성 ===')
day12 = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-12 (수)'}}]}
    },
}, method='POST')
day12_id = day12['id']

# 3/12 - 1. 실시간 추론 스크립트 고도화
api(f'https://api.notion.com/v1/blocks/{day12_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. 실시간 추론 스크립트 고도화'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'confidence 임계값 0.5 → 0.7으로 상향 (오탐 감소)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '해상도 640x480 → 1280x720 (더 정밀한 인식)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '깊이 필터링 추가: 1.5m 이상 depth 무시'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'temporal smoothing: 3프레임 연속 감지 시에만 표시 (깜빡임 방지)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'segmentation 마스크 오버레이 추가'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: realsense_detect.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealSense에서 RGB+Depth 프레임을 동시 캡처하여 YOLO seg 모델로 실시간 추론하는 스크립트. depth 필터링으로 먼 거리 오탐을 제거하고, temporal smoothing으로 일시적 오탐을 필터링함. 감지된 객체에 seg 마스크를 반투명 오버레이로 표시하며, 거리 정보를 함께 출력.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/12 - 2. 학습 하이퍼파라미터 강화
api(f'https://api.notion.com/v1/blocks/{day12_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. 학습 하이퍼파라미터 강화 (1차 목표 95%)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'epochs 150→200, patience 30→40'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'augmentation 강화: degrees 25, scale 0.5, mosaic 0.7, mixup 0.1, perspective 0.0005'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'HSV/translate/copy_paste 파라미터 강화'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: train_seg.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'YOLOv8s-seg 모델 학습 스크립트. 라벨이 있는 이미지만 필터링하여 80/20 split 후 학습. augmentation을 대폭 강화하여 다양한 각도/조명/크기 변화에 대한 일반화 성능 향상 목표. batch=32는 VRAM 8GB에서 스왑 발생하여 batch=16 유지.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/12 - 3. 스킵된 이미지 분석 및 후면 라벨링
api(f'https://api.notion.com/v1/blocks/{day12_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 스킵된 이미지 분석 및 후면 라벨링'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'auto_label 결과 중 2033장 스킵됨 (대부분 후면 촬영 → 기존 모델이 인식 못함)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '스킵 이미지를 별도 폴더로 복사하여 분석'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '후면 이미지 ~100장 수동 polygon 라벨링 (LabelMe)'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: copy_skipped.py / view_skipped.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'copy_skipped.py: 라벨이 없는(스킵된) 이미지를 별도 폴더로 복사하는 유틸리티. view_skipped.py: 스킵된 이미지를 순차적으로 표시하여 어떤 유형이 미탐지되는지 분석하는 뷰어.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/12 - 4. spike 라벨 수정 + 후면 학습
api(f'https://api.notion.com/v1/blocks/{day12_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. spike 라벨 수정 및 후면 모델 학습'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'polygon에서 중심점 대비 이상치 꼭짓점 자동 제거 (접시 마스크에서 선이 튀어나오는 현상 해결)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '후면 이미지 + 수동 라벨로 bowl_seg_v33 학습 진행'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '이 모델로 스킵된 2033장 자동 라벨링 예정'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: fix_spike_labels.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '각 polygon의 중심점을 계산하고, 중심에서의 거리가 평균 대비 표준편차 2배 이상인 꼭짓점을 이상치로 판단하여 자동 제거. 자동 라벨링 시 발생하는 spike(뾰족하게 튀어나온 꼭짓점) 문제를 일괄 수정.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

print(f'  3/12 작성 완료')

# ========================================
# 3월 13일 (목)
# ========================================
print('\n=== 2026-03-13 (목) 페이지 생성 ===')
day13 = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-13 (목)'}}]}
    },
}, method='POST')
day13_id = day13['id']

# 3/13 - 1. 영상 녹화 & 프레임 추출
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. RealSense 영상 녹화 & 프레임 추출'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '4개 영상 촬영 (정면/후면/좌측/우측) → 928장 프레임 추출'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'frame_interval=3 (30fps → 10fps) 적용'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: realsense_record.py / extract_frames.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'realsense_record.py: RealSense 카메라에서 1280x720 30fps mp4 영상 녹화. 스페이스바로 녹화 시작/정지. extract_frames.py: mp4에서 지정 간격(매 3프레임)으로 이미지를 추출하여 학습 데이터로 변환.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/13 - 2. 수동 라벨링 + Auto 라벨링 + 전체 학습
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. 수동 라벨링 → Auto 라벨링 → 전체 학습'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료 (평균 93% 달성)'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'labelme로 110장 polygon 수동 라벨링 → bowl_seg_v36 학습 (32 에포크 early stopping)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '원인: 영상 프레임 다양성 부족 (연속 프레임이 거의 동일), 데이터 110장으로 불충분'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '추가 50장 라벨링 후 bowl_seg_v37 재학습 → auto 라벨링 성공'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '전체 ~5000장 (기존 ~3000장 + 신규 ~2000장) 최종 학습 → 평균 인식률 93% 달성'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: auto_label_polygon.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '학습된 seg 모델로 미라벨 이미지에 자동 polygon 라벨링. conf 0.5 이상만 적용하고, 면적/꼭짓점 수/볼록도 기반 품질 필터링으로 저품질 라벨 자동 제외. 핵심 교훈: auto-label 품질은 수동 라벨의 다양성(최소 200장)에 직접 의존.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/13 - 3. 인식 결과 시각화 기능
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 인식 결과 시각화 기능 개발'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '녹화 기능: 인식률 표시 영상(mp4) + confidence 추이 그래프(png) + 구간 분석 리포트(txt)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '그래프: Avg/Max confidence, 90% 기준선, 미감지 구간 빨간 표시'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '리포트: 미감지/90%미만/안정 구간을 테이블로 정리'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: realsense_detect.py 녹화 모듈'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'w키로 녹화 시작, e키로 정지+저장. 녹화 중 매 프레임의 confidence를 기록하고, 종료 시 matplotlib으로 시계열 그래프 생성. 구간별 분석은 연속 프레임 기준으로 미감지/저신뢰/안정 구간을 분류하여 텍스트 리포트 출력. recordings/ 폴더에 타임스탬프 기반 파일명으로 저장.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/13 - 4. RealMan RM65 로봇 연결
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. RealMan RM65 로봇 연결 및 SDK 테스트'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '네트워크: 이더넷 192.168.1.100 (로봇) + WiFi (인터넷) 동시 사용'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SDK 연결 성공: Robotic_Arm v1.1.4, TCP/IP 192.168.1.18:8080'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'webapp으로 수동 제어 확인 → SDK 프로그래밍 제어 전환'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# 3/13 - 5. Eye-to-Hand 캘리브레이션
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '5. Eye-to-Hand 캘리브레이션 (step1~step4)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '진행 중 (개선 필요)'}, 'annotations': {'color': 'orange', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step1: RealSense + 로봇 연결 확인'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealSense RGB+Depth 스트리밍 + 로봇 TCP 읽기 동시 확인 → 정상'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step2: Depth 안정성 확인'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '0.42m 거리: 편차 ±2mm 이내'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '0.69m 거리: 편차 ±3mm 이내'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'spatial + temporal 필터 적용으로 안정적인 depth 확보'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step3: 캘리브레이션 데이터 수집 + 변환 행렬 계산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '로봇 TCP(플랜지 볼트)를 카메라에서 클릭 + 로봇 좌표 동시 기록'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SVD 기반 rigid transform (R, t) 계산 → 4x4 변환 행렬 저장'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '7포인트 수집 완료, 정확도 개선을 위해 포인트 수 증가 및 넓은 분포 필요'}}]}},
    ]
}, method='PATCH')

# 3/13 - 5 (계속) step4 + 시행착오
api(f'https://api.notion.com/v1/blocks/{day13_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step4: YOLO 접시 인식 → 좌표 변환 → 로봇 이동 검증'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'YOLO로 접시 인식 → 캘리브레이션 행렬로 로봇 좌표 변환 → Movej_P_Cmd로 이동'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '안전장치: SAFE_HEIGHT(접시 위 50mm 정지), MIN_Z(최소 높이 제한), 저속(v=5)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '시행착오 및 개선사항'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Movel_Cmd → Movej_P_Cmd 변경 (직선 경로에서 관절 걸림 발생)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '역기구학(IK) 해 없음 문제: 특정 위치+자세 조합에서 도달 불가 → webapp에서 실측한 자세값(rx,ry,rz) 적용'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 포인트 분포가 좁으면 변환 정확도 저하 → 넓은 영역에서 수집 필요'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '코드 리뷰: step3_calibration.py / step4_verify.py'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step3: 카메라 화면에서 마우스 클릭으로 3D 좌표를 취득하고, 동시에 로봇 TCP 좌표를 읽어 쌍으로 저장. N개 포인트가 모이면 SVD로 최적 회전행렬(R)과 이동벡터(t)를 계산하여 4x4 변환 행렬로 저장. step4: YOLO seg 모델로 접시를 인식하고, 중심점의 depth로 3D 좌표를 구한 뒤 변환 행렬을 적용하여 로봇 좌표로 변환. 고정 자세(rx,ry,rz)와 안전 높이를 더해 Movej_P_Cmd로 이동 명령 전송.'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

print(f'  3/13 작성 완료')

print('\n완료!')
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       ├── 2026-03-12 (수)')
print('       └── 2026-03-13 (목)')
