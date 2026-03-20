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
# 3월 18일 (수)
# ========================================
print('\n=== 2026-03-18 (수) 페이지 생성 ===')
day18 = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': sw_page_id},
    'icon': {'type': 'emoji', 'emoji': '\U0001f4dd'},
    'properties': {
        'title': {'title': [{'text': {'content': '2026-03-18 (수)'}}]}
    },
}, method='POST')
day18_id = day18['id']

# ── 1. 카메라 각도 측정 ──
print('  1. 카메라 각도 측정 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '1. 카메라 기울기 각도 측정 (step2_depth_check.py)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '테이블 위 클릭 포인트들로 평면 fitting → 법선 벡터 → 카메라 기울기 계산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '측정 결과: 수직 기준 55~56° (depth 기반, 여러 차례 측정 일관)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '줄자 측정(수직 45cm, 수평 43cm → 43.7°)과 차이 → depth 기반이 렌즈 기준이라 더 정확'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '최종 확정: CAMERA_TILT_DEG = 55.0°'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '시행착오'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '100x100 픽셀 영역 평면 fitting → 57.2° (depth 노이즈로 부정확)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '클릭 2점 거리 기반 → 79.9° (3D 좌표 계산 오류)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '최종: 마우스 위치 주변 depth 샘플(~260개)로 평면 fitting → 55~56° 안정적'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 2. 접시 중심 검출 개선 과정 (6가지 시도) ──
print('  2. 접시 중심 검출 개선 과정 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '2. 접시 중심 검출 정확도 개선 (6가지 시도)'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '실용적 수준 달성 (오프셋 보정 적용)'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라가 수직에서 55° 기울어져 있어 접시 중심 검출이 편향되는 문제. verify.py에서 4가지 방법을 시도하고, test.py에서 2가지 추가 보정을 적용하여 최종적으로 실용적 수준에 도달.'}}]}},
    ]
}, method='PATCH')

# 방법 1~3
print('    방법 1~3 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 1: bbox 중심 (verify.py) → 실패'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'YOLO detection box의 (x1+x2)/2, (y1+y2)/2'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '실패 원인: 55° 기울기에서 bbox가 비대칭 → 중심이 우측으로 편향'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 2: seg 마스크 무게중심 (verify.py) → 개선되었으나 부족'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'cv2.moments로 마스크 면적 기반 centroid 계산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '한계: 카메라에 가까운 쪽 픽셀이 더 많아 카메라 방향으로 편향'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 3: ellipse fitting (verify.py) → 개선되었으나 부족'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'cv2.fitEllipse로 기하학적 타원 중심 계산'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '한계: moments보다 나으나 seg 마스크 자체가 카메라 각도로 비대칭'}}]}},
    ]
}, method='PATCH')

# 방법 4~6
print('    방법 4~6 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 4: 3D contour center (verify.py 최종) → 개선되었으나 ~2cm 오차'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '윤곽점 50개를 각각 3D deproject → 3D 좌표 평균'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라 각도에 독립적이지만 각 점마다 다른 depth → depth 노이즈에 취약'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '결과: 아래 2cm + 우측 2cm 오차 (위치 바꿔도 동일한 방향/거리)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 5: ellipse center + rim depth + 기울기 보정 (test.py) → 개선'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'ellipse fitting으로 2D 중심 → 윤곽 테두리 depth 25th percentile(rim depth) 사용'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 깊이 30mm + 55° 기울기 기반 기하학적 보정 추가'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '중심 depth(접시 안쪽 바닥) 대신 rim depth 사용으로 3D 좌표 정확도 향상'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '결과: 여전히 ~2cm 오차 (기울기 보정만으로는 부족)'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '방법 6: 고정 오프셋 보정 (test.py 최종) → 실용적 수준 달성'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 위치를 바꿔도 같은 방향/거리 오차 → 고정 오프셋으로 해결 가능 확인'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '로봇 좌표에 OFFSET_X_MM, OFFSET_Y_MM 추가 (cam_to_robot 함수에서 적용)'}}]}},
    ]
}, method='PATCH')

# 오프셋 시행착오
print('    오프셋 시행착오 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '오프셋 부호 결정 시행착오'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'X=-20, Y=+20 → 로봇 리치 한계로 이동 실패 (X가 -435mm까지 감)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'X=0, Y=0 → 이동 성공, 여전히 2cm 오차 (기준선 확인용)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'X=-20, Y=+20 → Y(하단)는 중심으로 옴, X(우측)는 4cm로 오차 증가 → X 부호 맞음, Y 부호 반대'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'X=-20, Y=-20 → 성공, 오차 감소하여 실용적 수준 달성'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '최종 설정: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'OFFSET_X_MM = -20, OFFSET_Y_MM = -20'}},
            ]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 3. 근본 원인 분석 ──
print('  3. 근본 원인 분석 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '3. 근본 원인 분석'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라가 55°(수직 기준)로 기울어져 있으면, seg 마스크의 경계가 접시의 물리적 rim과 일치하지 않음. 가까운 쪽 rim이 더 넓게, 먼 쪽이 좁게 보이기 때문에 어떤 2D/3D 방법을 써도 카메라 각도에서 오는 마스크 편향은 남음.'}}]}},
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '해결 방향'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '근본 해결: 카메라를 더 수직(top-down)으로 설치 → 편향 자체를 줄임'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '현실적 해결: 고정 오프셋 보정 (현재 적용 중) + 흡착 그리퍼가 어느 정도 커버'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 4. step5_test.py 최종 구성 ──
print('  4. step5_test.py 최종 구성 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '4. step5_test.py 최종 구성'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'CAMERA_TILT_DEG = 55.0° (depth 기반 측정)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'OFFSET_X_MM = -20, OFFSET_Y_MM = -20 (고정 오프셋 보정)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Depth: 윤곽 테두리 40개 샘플 → 25th percentile (rim depth)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 중심: ellipse fitting 2D 중심 + rim depth로 3D deproject'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '기울기 보정: bowl_depth 30mm × sin(55°) × 0.5 → 카메라 Y/Z축 보정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '로봇 속도: v=20 (Movej_P_Cmd)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SAFE_HEIGHT = 200mm, MIN_Z = 80mm'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 향후 계획 ──
print('  향후 계획 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '향후 계획'}}]}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '오프셋 미세 조정 (다양한 위치에서 반복 테스트)'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '그리퍼 장착 후 실제 pick & place 테스트'}}],
            'checked': False}},
        {'object': 'block', 'type': 'to_do', 'to_do': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라를 더 수직으로 재설치 검토 (편향 근본 해결)'}}],
            'checked': False}},
    ]
}, method='PATCH')

print('\n완료!')
print('프로젝트 진행 일지')
print('  └── Samsung Wellstory')
print('       └── 2026-03-18 (수)')
