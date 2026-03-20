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
        print(f'ERROR {e.code}: {e.read().decode("utf-8")[:500]}')
        raise

# Samsung Wellstory 페이지 찾기
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

# 2026-03-19 페이지 생성
print('\n=== 2026-03-19 (목) 페이지 생성 ===')
day19 = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '📝'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-19 (목)'}}]}
    },
}, method='POST')
day19_id = day19['id']

# ── 1. step7 Code Review Notion 정리 ──
print('  1. step7 Code Review 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day19_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. step7_twoplate_alwaysdetect.py Code Review Notion 정리'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '프로젝트 진행 일지 내 Code Review 페이지 신설 (계층 구조: Code Review → step7 / step3 서브페이지)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'step7 전체 코드 업로드 + 7개 섹션 상세 설명 작성 (초보자 기준)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '설명 항목 (7개 섹션)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '전체 동작 흐름: s키(Start 저장) → g키(연속 루프 시작) → 자동 접시 인식 및 순차 이동 → x키(중지)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 행렬: calibration_matrix.npy를 통한 카메라 → 로봇 좌표 변환 원리 (R, t 분리 설명)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 중심 검출: 타원 피팅 → rim depth(25th percentile) → 55° 기울기 보정 → 오프셋(X=-20, Y=+25mm)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 우선순위: YOLO로 인식된 접시를 Start Point 기준 거리 순 정렬 → 가장 가까운 접시부터 순차 방문 (dist_from_start 함수)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '교대 방문 로직: last_visited_rob_xyz로 직전 방문 접시 위치 기억 → 다음 사이클에서 해당 접시 제외하고 선택 (1→2→1→2 반복)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '대기 루프: 접시 없을 시 YOLO 재인식 대기 (500ms 간격) → 접시 재배치 감지 시 자동 재개, last_visited 초기화'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '로봇 이동: Movej_Cmd(Start 복귀, 관절각) + Movej_P_Cmd(접시 이동, Cartesian) 조합 이유 설명'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '오프셋 보정: OFFSET_X=-20mm, OFFSET_Y=+25mm 조정 방법 및 부호 판단 기준'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 2. step3 Code Review Notion 정리 ──
print('  2. step3 Code Review 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day19_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. step3_calibration.py Code Review Notion 정리'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Code Review 하위에 step3 서브페이지 추가 (전체 코드 + 7개 섹션 설명)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '주요 설명 항목'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Eye-to-Hand 개념: 고정 카메라 ↔ 로봇 베이스 좌표계 차이 및 변환 행렬 T 역할'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SVD 기반 변환 행렬 계산: 중심화 → H 행렬 → SVD → R/t 도출 단계별 설명'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '빨간 마커 HSV 자동 보정: H 0~10 + 170~180 두 범위, S 임계값 30 (조명 변화 대응), 원형도 필터'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'depth 5x5 중앙값: 단일 픽셀 노이즈 방지, 이상치에 강한 중앙값 사용 이유'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'x키 오차 포인트 자동 삭제: 10mm 이상 오차 역순 삭제 후 재계산으로 정확도 향상'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '정확도 팁: 포인트 8~10개, 작업 범위 전체 골고루 분포, 목표 평균 5mm 이내'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 향후 계획 ──
print('  향후 계획 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day19_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '향후 계획'}}]}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '오프셋 미세 조정 (다양한 위치 반복 테스트)'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '그리퍼 장착 후 실제 pick & place 테스트'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': 'MoveL 정상 작동 확인 후 pick & place 동작 적용 검토'}}],
            'checked': False}},
    ]
}, method='PATCH')

print('\n완료!')
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       └── 2026-03-19 (목)')
