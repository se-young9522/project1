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

# 3월 13일 서브페이지 생성
print('3월 13일 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': march_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '26.03.13 (목)'}}]}
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
            'rich_text': [{'type': 'text', 'text': {'content': '데이터 학습 결과'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 전체 ~5000장 학습 완료, 평균 인식률 93% 달성'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '인식 결과 시각화 기능 개발'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 모델 경로 수정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 녹화 기능 추가: 인식률 영상(mp4) + confidence 그래프(png) + 구간 분석 리포트(txt)'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealMan RM65 로봇 연결'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 네트워크 설정 (이더넷 192.168.1.100 + WiFi 동시)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> SDK 연결 테스트 성공 (Robotic_Arm, TCP/IP 192.168.1.18)'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Eye-to-Hand 캘리브레이션 (step1~step4)'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step1: RealSense + 로봇 연결 확인'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step2: Depth 안정성 확인 (0.42m: ±2mm, 0.69m: ±3mm)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step3: 캘리브레이션 데이터 수집 (7포인트), 변환 행렬 계산'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step4: YOLO 접시 인식 → 좌표 변환 → 로봇 이동 검증'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 시행착오: 포인트 분포, IK 도달 불가, 자세값 조정 등 반복 개선'}}]}},
    ]
}, method='PATCH')

print('\n완료!')
print('일일 업무 보고')
print('  └── 2026년 3월')
print('       └── 26.03.13 (목)')
