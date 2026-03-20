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
    print('3월 페이지를 찾을 수 없습니다.')
    sys.exit(1)
print(f'3월 페이지 ID: {march_page_id}')

# 3월 18일 서브페이지 생성
print('3월 18일 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': march_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '26.03.18 (수)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']

# 내용 작성
print('내용 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_2', 'heading_2': {
            'rich_text': [{'type': 'text', 'text': {'content': '삼성웰스토리'}}]}},

        # 1. 접시 인식 간이 공정 세팅
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 인식 간이 공정 세팅'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> RealSense D435i 카메라 고정 설치 (비스듬 55° 각도)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 로봇팔(RM65) 작업 범위 내 접시 배치'}}]}},

        # 2. Eye-to-Hand 캘리브레이션 재수행
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Eye-to-Hand 캘리브레이션 재수행'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 3/17 결과 (9포인트, Y축 -40mm 오차) → 포인트 분포 개선하여 재수행'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 7포인트 수집 → 평균 5.3mm / 최대 7.8mm 달성 (calibration_matrix.npy 저장)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 카메라 위치가 변경될 때마다 캘리브레이션 재수행 필요 → 매일 세팅 시 캘리브레이션 진행 (최종 5mm 이내 목표)'}}]}},

        # 3. 접시 중심 검출 보정
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 중심 검출 보정'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 카메라 기울기 55° (수직 기준) depth 기반 측정 확정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 카메라 비스듬 설치로 seg 마스크 편향 발생 → 고정 오프셋(X=-20, Y=-20mm) 적용하여 실용적 수준 달성'}}]}},

        # 4. Start→Bowl→Start 자동 테스트
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step5_test.py Start→Bowl→Start 자동 테스트 구현'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> s키: Start Point 저장, g키: Start→접시 위(200mm)→Start 자동 왕복 성공'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 접시 인식 후 이동 결과: 목표 vs 실제 TCP 오차 1mm 미만'}}]}},
    ]
}, method='PATCH')

print('\n완료!')
print('일일 업무 보고')
print('  └── 2026년 3월')
print('       └── 26.03.18 (수)')
