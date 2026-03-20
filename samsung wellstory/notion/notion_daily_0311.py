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
    print(f'  발견: {title} ({page["id"]})')
    if '일일 업무 보고' in title:
        daily_page_id = page['id']
        break

if not daily_page_id:
    print('일일 업무 보고 페이지를 찾을 수 없습니다.')
    sys.exit(1)

print(f'일일 업무 보고 페이지 ID: {daily_page_id}')

# 3월 페이지 찾기/생성
print('3월 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{daily_page_id}/children?page_size=100')
march_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        print(f'  발견: {title}')
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

# 3월 11일 서브페이지 생성
print('3월 11일 페이지 생성...')
day_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': march_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '26.03.11 (화)'}}]}
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
            'rich_text': [{'type': 'text', 'text': {'content': 'rectangle → polygon 라벨링 전환'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 기존 rectangle 라벨링 2900장 training 결과 학습 이미지 인식률 95% 이상'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 그러나 실물 접시 테스트 시 정면만 인식 가능, 측면/후면 인식 불가'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 이를 보완하기 위해 polygon 라벨링으로 전환'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 수동 polygon 라벨링 130장 완료 후 1차 seg 모델 학습'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 학습된 모델로 나머지 ~2800장 자동 polygon 라벨링 수행'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '자동 라벨링 결과 필터링'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 자동 라벨링 시 다중 polygon 감지된 917개 파일에서 가장 큰 polygon만 유지'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 그릇 외 영역(노트북 화면 등) 오탐 제거'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '라벨 검수 및 오탐 삭제'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> verify_labels.py로 전체 라벨 시각적 검수'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 비정상 라벨 61개 식별 후 삭제 (polygon 선 이탈, 노트북 화면 오탐)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> 최종 유효 라벨: ~2869장'}}]}},

        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '재학습 스크립트 수정 (train_seg.py)'}, 'annotations': {'bold': True}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> epochs 100→150, patience 20→30으로 조정'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> augmentation 추가 (mosaic, hsv, translate, copy_paste)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '-> batch 32 시도 → VRAM 부족으로 batch 16 복원'}}]}},

        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': ''}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '*금일 라벨 정리 완료, 명일 전체 ~2869장 재학습 실행 예정*'}, 'annotations': {'italic': True}}]}},
    ]
}, method='PATCH')

print('\n완료!')
print('일일 업무 보고')
print('  └── 2026년 3월')
print('       └── 26.03.11 (화)')
