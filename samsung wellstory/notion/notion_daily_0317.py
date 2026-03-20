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

# 일일 업무 보고 페이지 찾기
print('일일 업무 보고 페이지 검색 중...')
search = api('https://api.notion.com/v1/search', {
    'query': '일일 업무 보고',
    'filter': {'property': 'object', 'value': 'page'},
}, method='POST')

daily_page_id = None
for page in search.get('results', []):
    title = ''
    for t in page.get('properties', {}).get('title', {}).get('title', []):
        title += t.get('plain_text', '')
    if '일일 업무 보고' in title:
        daily_page_id = page['id']
        break

if not daily_page_id:
    print('일일 업무 보고 페이지를 찾을 수 없습니다.')
    sys.exit(1)
print(f'일일 업무 보고 페이지 ID: {daily_page_id}')

# 3월 페이지 찾기
print('3월 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{daily_page_id}/children?page_size=100')
march_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        if '3월' in title or 'March' in title:
            march_page_id = block['id']
            break

if not march_page_id:
    print('3월 페이지 생성...')
    march_page = api('https://api.notion.com/v1/pages', {
        'parent': {'page_id': daily_page_id},
        'icon': {'type': 'emoji', 'emoji': '\U0001f4c5'},
        'properties': {
            'title': {'title': [{'text': {'content': '2026년 3월'}}]}
        },
    }, method='POST')
    march_page_id = march_page['id']

print(f'3월 페이지 ID: {march_page_id}')

# 3월 17일 서브페이지 생성
print('3월 17일 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': march_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '26.03.17 (화)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']

# 내용 작성
print('내용 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_2', 'heading_2': {
            'rich_text': [{'type': 'text', 'text': {'content': '삼성웰스토리'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Eye-to-Hand 캘리브레이션 코드 개선 (step3_calibration.py)'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> RealSense D435i 최적화: 레이저 파워 최대화, depth 필터 튜닝 (spatial/temporal/hole filling)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 안정화 프레임 30→60 (auto-exposure 수렴 대기)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 클릭 시 5x5 영역 depth 중앙값 사용 (단일 픽셀 노이즈 방지)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 최소 depth 임계값 0.1→0.2m (D435i 최소 측정거리 0.28m 고려)'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '마커 중심 자동 검출 기능 추가'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 엔드툴 볼트에 빨간색 마커 부착 후 HSV 색상 기반 검출'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 160x160 crop 영역에서 빨간 마커 무게중심(centroid) 자동 보정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 검출 실패 시 클릭 위치 fallback 처리'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 포인트 관리 기능 개선'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 포인트 삭제 시 번호 지정 삭제 기능 추가 (기존: 마지막만 삭제 가능)'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 테스트 및 검증'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 9포인트 수집 후 캘리브레이션 수행'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> Y축 -40mm 오차 확인 → Y방향 포인트 분포 부족이 원인'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> TCP 좌표 기준 오차 5mm 이내 달성'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step4_verify.py 접시 인식 정확도 개선'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> YOLO bounding box 중심 → seg 마스크 무게중심으로 변경'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 접시의 실제 중심을 더 정확하게 계산하도록 개선'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '유사 프로젝트 조사 및 분석'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 참조 프로젝트 3건 분석 (KUKA+YOLOv4, PKUAILab Eye-to-Hand, 4DoF OpenCV)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> Pick & Place 자동화 루프 패턴 정리 (흡착→이동→배치→복귀)'}}]}},

        {'object': 'block', 'type': 'divider', 'divider': {}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '향후 계획'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 카메라 고정 개선 (C클램프 + L브라켓)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 캘리브레이션 재수행 (10포인트, X/Y/Z 넓은 분포)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 그리퍼 장착 후 실제 pick & place 테스트'}}]}},
    ]
}, method='PATCH')

print('\n완료!')
print('일일 업무 보고')
print('  └── 2026년 3월')
print('       └── 26.03.17 (화)')
