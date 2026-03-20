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
        print(f'ERROR {e.code}: {e.read().decode("utf-8")[:300]}')
        raise

# 일일 업무 보고 → 3월 페이지 찾기
print('3월 페이지 검색 중...')
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

children = api(f'https://api.notion.com/v1/blocks/{daily_page_id}/children?page_size=100')
march_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        if '3월' in title or 'March' in title:
            march_page_id = block['id']
            break

print(f'3월 페이지 ID: {march_page_id}')

# 3월 19일 서브페이지 생성
print('3월 19일 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': march_page_id},
    'icon': {'type': 'emoji', 'emoji': '📝'},
    'properties': {
        'title': {'title': [{'text': {'content': '26.03.19 (목)'}}]}
    },
}, method='POST')
day_page_id = day_page['id']

# 내용 작성
print('내용 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day_page_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_2', 'heading_2': {
            'rich_text': [{'type': 'text', 'text': {'content': '삼성웰스토리'}}]}},

        # 1. step7 Code Review Notion 정리
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step7_twoplate_alwaysdetect.py Code Review Notion 정리'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 프로젝트 진행 일지 > Code Review 페이지 구조 개편 (단일 페이지 → 계층 구조)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step7 전체 코드 + 7개 섹션 상세 설명 업로드 (초보자 기준 설명)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 설명 항목: 전체 동작 흐름 / 캘리브레이션 행렬 / 접시 중심 검출 / 연속 루프 구조 / 대기 루프 / 로봇 이동 명령 / 오프셋 보정 방법'}}]}},

        # 2. MoveL 동작 이슈 분석
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'MoveL 동작 이슈 분석'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> MoveL 실패 원인: 특이점(Singularity) — 직선 경로 중간에 도달 불가 구간 존재'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 제로자세(0,0,0,0,0,0)는 손목 3관절 일직선 → 특이점이므로 MoveL 사용 금지 확인'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> step7에서 Movej_P_Cmd 사용하는 이유: 특이점 회피 (TCP 목표는 Cartesian, 경로는 관절 공간)'}}]}},

        # 3. movel_test.py
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'movel_test.py 작성 및 MoveL 정상 작동 확인'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 홈 자세 [0,0,90,0,90,0] 기준 Z축 MoveL 테스트 스크립트 작성 (h/z/b/q 키)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 테스트 결과: Z -50mm MoveL 정상 작동 확인 (관절각 변화로 실제 이동 검증)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 복귀(b키) 시 blocking 파라미터 미작동 이슈 → time.sleep(4)로 해결'}}]}},
    ]
}, method='PATCH')

print('\n완료!')
print('일일 업무 보고')
print('  └── 2026년 3월')
print('       └── 26.03.19 (목)')
