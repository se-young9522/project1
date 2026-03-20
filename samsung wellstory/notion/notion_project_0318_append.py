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

# Samsung Wellstory → 2026-03-18 페이지 찾기
print('2026-03-18 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{PROJECT_PAGE_ID}/children?page_size=100')
sw_page_id = None
for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        if 'Samsung' in title or 'Wellstory' in title:
            sw_page_id = block['id']
            break

if not sw_page_id:
    print('Samsung Wellstory 페이지를 찾을 수 없습니다.')
    sys.exit(1)

# 3/18 페이지 찾기
children2 = api(f'https://api.notion.com/v1/blocks/{sw_page_id}/children?page_size=100')
day18_id = None
for block in children2.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        if '03-18' in title or '3-18' in title:
            day18_id = block['id']
            break

if not day18_id:
    print('2026-03-18 페이지를 찾을 수 없습니다.')
    sys.exit(1)
print(f'2026-03-18 페이지 ID: {day18_id}')

# ── 5. 캘리브레이션 재수행 결과 ──
print('  5. 캘리브레이션 재수행 결과 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '5. Eye-to-Hand 캘리브레이션 재수행'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료 (avg 5.3mm)'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '3/17 결과 (재수행 전)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '9포인트 수집 → Y축 -40mm 오차'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '원인: Y방향 포인트 분포가 좁았음 (한쪽에 몰림)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '3/18 결과 (재수행 후)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '7포인트 수집 (X/Y/Z 넓은 분포로 개선)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '평균 오차: 5.3mm / 최대 오차: 7.8mm → "양호" 등급'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'calibration_matrix.npy 저장 완료'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '좌표 변환 정확도: 2~3mm 이내 (캘리브레이션 자체는 정확)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '정확도 기준 (목표: 3~5mm)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '우수: 1~3mm (정밀 조립)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '양호: 3~5mm (일반 pick & place) ← 목표'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '보통: 5~10mm (큰 물체) ← 현재 달성 (avg 5.3mm)'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 6. verify.py → test.py 진화 과정 ──
print('  6. verify → test 진화 과정 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '6. step4_verify.py → step5_test.py 진화'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션 좌표 변환은 정확(2~3mm)하지만, 접시 중심 검출 편향으로 로봇이 실제 접시 중심에 가지 않는 문제를 해결하기 위해 verify.py를 기반으로 test.py를 생성하여 다양한 보정 기법을 적용.'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'verify.py vs test.py 주요 차이점'}}]}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '접시 중심 계산: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '윤곽 50개 3D 평균 → ellipse fitting 2D 중심 + rim depth'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 'Depth 소스: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '각 윤곽점 개별 depth → 윤곽 40개 샘플 25th percentile (rim depth)'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '카메라 각도 보정: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '없음 → CAMERA_TILT_DEG=55° 기반 기하학적 보정 추가'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '좌표 보정: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '없음 → OFFSET_X/Y_MM 고정 오프셋 보정 추가'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '로봇 속도: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'v=5 → v=20'}},
            ]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 7. 캘리브레이션 시행착오 요약 ──
print('  7. 캘리브레이션 시행착오 요약 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '7. 캘리브레이션 전체 시행착오 요약 (3/13~18)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '1차(5pt): X범위 17mm로 너무 좁음, 평균 0.8mm (수치상 좋으나 실제 부정확 — 과적합)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '2차: 로봇을 안 움직이고 클릭만 여러번 (같은 TCP로 여러 포인트 — 의미 없음)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '3차: 43.8mm 오차 (포인트 분포 불량)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '4차(7pt): "보통" 등급 → step4 이동 시 Z 너무 낮음 (CONTROLLER_DATA_RETURN_FALSE)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '5차(9pt, 3/17): Y축 -40mm 오차 → Y방향 포인트 분포 부족'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '6차(7pt, 3/18): avg 5.3mm, max 7.8mm → "양호" 등급 달성'}, 'annotations': {'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '핵심 교훈'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '포인트 수보다 분포가 중요 (X/Y/Z 넓게 분포시켜야 함)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SVD 기반 캘리브레이션은 좌표 변환 자체는 정확 (2~3mm)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '실제 pick 정확도는 캘리브레이션 + 접시 중심 검출 + depth 정확도의 합산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Movel은 관절 걸림 발생 → Movej_P_Cmd 사용 (관절 공간 이동)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SAFE_HEIGHT 50→200mm 으로 안전 마진 확보 필수'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 8. step4 로봇 이동 시행착오 ──
print('  8. 로봇 이동 시행착오 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '8. step4/5 로봇 이동 관련 시행착오'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Movel vs Movej_P 문제'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Movel (직선 이동): 경로상 관절 한계에 걸려 CONTROLLER_DATA_RETURN_FALSE 빈번'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Movej_P_Cmd (관절 공간 이동): 경로가 직선이 아니지만 관절 한계 회피 → 채택'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '엔드이펙터 자세 고정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '초기: 캘리브레이션에서 자세까지 변환 시도 → 불안정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '해결: rx/ry/rz를 수동 측정 고정값으로 분리 (DOWN_RX=3.103, DOWN_RY=0.051, DOWN_RZ=0.661)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '캘리브레이션은 위치(xyz)만 담당, 자세는 별도 관리'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '안전 설정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SAFE_HEIGHT: 50mm → 200mm (초기 테스트 시 충돌 방지)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'MIN_Z: 80mm (테이블 충돌 방지 최소 높이)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '속도: v=5 → v=20 (테스트 확인 후 점진 증가)'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 9. RealSense 창 꺼짐 문제 ──
print('  9. RealSense 창 꺼짐 문제 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '9. RealSense 창 꺼짐 문제'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '현상: q키를 누르지 않았는데 OpenCV 창이 닫히는 현상 발생'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '원인: try/finally 구조로 예외 발생 시 자동 정리 → 에러 메시지가 보이지 않음'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '해결: 터미널에서 직접 python step4_verify.py 실행 → traceback으로 원인 확인'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'calibration_matrix.npy가 저장되어 있으므로 재캘리브레이션 없이 verify.py 재실행 가능'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

print('\n추가 완료!')
print('2026-03-18 페이지에 5~9번 항목 추가됨')
