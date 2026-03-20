"""
Notion - 로봇 1차 calibration 참조 페이지 생성
RealMan 페이지 > RealSense + 로봇팔 객체인식 Pick & Place 단계별 확인 > 하위 페이지로 생성
"""
import json, urllib.request, sys, time
sys.stdout.reconfigure(encoding='utf-8')

API_KEY = 'ntn_b57077336502qA3Deg7jq4qWxqnc8navEmCPcNhjyv7gn7'
RM_PAGE_ID = '32115435-1a92-81d0-816a-f016c4cecf14'  # RealMan 페이지

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

def text(content, bold=False, italic=False, color='default', code=False):
    annotations = {}
    if bold: annotations['bold'] = True
    if italic: annotations['italic'] = True
    if color != 'default': annotations['color'] = color
    if code: annotations['code'] = True
    return {'type': 'text', 'text': {'content': content}, 'annotations': annotations} if annotations else {'type': 'text', 'text': {'content': content}}

def heading(level, content):
    key = f'heading_{level}'
    return {'object': 'block', 'type': key, key: {'rich_text': [text(content)]}}

def paragraph(*rich_texts):
    return {'object': 'block', 'type': 'paragraph', 'paragraph': {'rich_text': list(rich_texts)}}

def bullet(content):
    return {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {'rich_text': [text(content)]}}

def bullet_rich(*rich_texts):
    return {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {'rich_text': list(rich_texts)}}

def numbered(content):
    return {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {'rich_text': [text(content)]}}

def code_block(content, lang='plain text'):
    if len(content) > 2000:
        content = content[:1997] + '...'
    return {'object': 'block', 'type': 'code', 'code': {
        'rich_text': [text(content)], 'language': lang}}

def divider():
    return {'object': 'block', 'type': 'divider', 'divider': {}}

def callout(content, emoji='📌'):
    return {'object': 'block', 'type': 'callout', 'callout': {
        'rich_text': [text(content)],
        'icon': {'type': 'emoji', 'emoji': emoji},
    }}

def table_row(cells):
    return {'object': 'block', 'type': 'table_row', 'table_row': {
        'cells': [[text(c)] for c in cells]
    }}

def table(width, rows):
    return {'object': 'block', 'type': 'table', 'table': {
        'table_width': width,
        'has_column_header': True,
        'has_row_header': False,
        'children': [table_row(r) for r in rows],
    }}

def append_blocks(page_id, children):
    for i in range(0, len(children), 100):
        batch = children[i:i+100]
        api(f'https://api.notion.com/v1/blocks/{page_id}/children', {
            'children': batch
        }, method='PATCH')
        time.sleep(0.3)

# ──────────────────────────────────────────────────────────────
# Step 1: RealMan 페이지에서 "RealSense + 로봇팔..." 하위 페이지 찾기
# ──────────────────────────────────────────────────────────────
print('RealMan 페이지에서 하위 페이지 검색 중...')
children = api(f'https://api.notion.com/v1/blocks/{RM_PAGE_ID}/children?page_size=100')
pick_place_page_id = None

for block in children.get('results', []):
    if block['type'] == 'child_page':
        title = block.get('child_page', {}).get('title', '')
        print(f'  발견: {title}')
        if 'Pick' in title or 'Place' in title or '단계별' in title or 'RealSense' in title:
            pick_place_page_id = block['id']
            print(f'  → 대상 페이지 발견: {title} ({pick_place_page_id})')
            break

# 못 찾으면 RealMan 페이지 직접 하위로 생성
parent_id = pick_place_page_id if pick_place_page_id else RM_PAGE_ID
parent_name = '단계별 확인 페이지' if pick_place_page_id else 'RealMan 페이지'
print(f'\n"{parent_name}" 밑에 하위 페이지 생성합니다.')

# ──────────────────────────────────────────────────────────────
# Step 2: "로봇 1차 calibration 참조" 하위 페이지 생성
# ──────────────────────────────────────────────────────────────
print('\n"로봇 1차 calibration 참조" 페이지 생성 중...')

new_page = api('https://api.notion.com/v1/pages', {
    'parent': {'page_id': parent_id},
    'properties': {
        'title': {
            'title': [text('로봇 1차 calibration 참조')]
        }
    },
    'icon': {'type': 'emoji', 'emoji': '🔧'},
}, method='POST')

cal_page_id = new_page['id']
print(f'  페이지 생성 완료: {cal_page_id}')

# ──────────────────────────────────────────────────────────────
# Section 1: 최종 캘리브레이션 구성
# ──────────────────────────────────────────────────────────────
print('\n섹션 1: 시스템 구성...')

system_diagram = """        [고정 RealSense D435i]  ← Top-Down (수직) 설치 권장
              │
     ─────────┼─────────
     │                  │
 [로봇팔 1]         [로봇팔 2]
 (RM65)              (RM65)
     │                  │
 [흡착 그리퍼]     [흡착 그리퍼]
              │
       [컨베이어 벨트]
      ◯ ◯ ◯ ◯ ◯ ◯ ◯ ◯  ← 접시들"""

append_blocks(cal_page_id, [
    heading(1, '최종 캘리브레이션 구성'),
    divider(),
    heading(2, '시스템 구성'),
    code_block(system_diagram),
])

# ──────────────────────────────────────────────────────────────
# Section 2: 캘리브레이션 방식
# ──────────────────────────────────────────────────────────────
print('섹션 2: 캘리브레이션 방식...')

append_blocks(cal_page_id, [
    heading(2, '캘리브레이션 방식: Eye-to-Hand x 2'),
    table(2, [
        ['항목', '내용'],
        ['카메라', '고정 RealSense 1대'],
        ['방식', 'Eye-to-Hand (카메라 고정, 로봇이 움직임)'],
        ['횟수', '로봇팔당 1회씩, 총 2회'],
        ['결과물', 'robot1_matrix.npy, robot2_matrix.npy'],
        ['기존 코드', 'step3_calibration.py 그대로 사용 가능'],
    ]),
])

# ──────────────────────────────────────────────────────────────
# Section 3: 캘리브레이션 절차
# ──────────────────────────────────────────────────────────────
print('섹션 3: 캘리브레이션 절차...')

cal_procedure = """1. 카메라 설치 + 고정 (Top-Down 권장)
   └─ 한번 고정하면 움직이지 않음

2. 로봇팔 1 캘리브레이션 (기존 step3과 동일)
   └─ 빨간 마커를 로봇팔1 TCP에 부착
   └─ 7~10개 포인트 수집 (카메라 좌표 ↔ 로봇1 좌표)
   └─ SVD로 변환행렬 계산 → robot1_matrix.npy 저장

3. 로봇팔 2 캘리브레이션 (동일 과정 반복)
   └─ 빨간 마커를 로봇팔2 TCP에 부착
   └─ 7~10개 포인트 수집 (카메라 좌표 ↔ 로봇2 좌표)
   └─ SVD로 변환행렬 계산 → robot2_matrix.npy 저장"""

append_blocks(cal_page_id, [
    heading(2, '캘리브레이션 절차'),
    code_block(cal_procedure),
])

# ──────────────────────────────────────────────────────────────
# Section 4: 운영 시 좌표 변환 흐름
# ──────────────────────────────────────────────────────────────
print('섹션 4: 좌표 변환 흐름...')

coord_flow = """카메라 프레임 (실시간)
    │
    ├─ YOLO seg → 접시 20개 검출 (픽셀 좌표)
    │
    ├─ depth + deproject → 카메라 3D 좌표 (mm)
    │
    ├─ 접시 분배 (왼쪽 → 로봇1, 오른쪽 → 로봇2)
    │
    ├─ robot1_matrix 적용 → 로봇1 좌표계 (mm)
    │                         + 오프셋 보정
    │
    └─ robot2_matrix 적용 → 로봇2 좌표계 (mm)
                              + 오프셋 보정"""

append_blocks(cal_page_id, [
    heading(2, '운영 시 좌표 변환 흐름'),
    code_block(coord_flow),
])

# ──────────────────────────────────────────────────────────────
# Section 5: Top-Down 설치 시 달라지는 점
# ──────────────────────────────────────────────────────────────
print('섹션 5: Top-Down 비교...')

append_blocks(cal_page_id, [
    heading(2, 'Top-Down 설치 시 달라지는 점'),
    table(3, [
        ['항목', '현재 (55°)', 'Top-Down (0°)'],
        ['기울기 보정', '필요 (CAMERA_TILT_DEG=55)', '불필요 (삭제)'],
        ['고정 오프셋', 'X=-20, Y=-20mm', '거의 0 (재측정 필요)'],
        ['seg 마스크 편향', '있음 (근본 한계)', '없음 (원형 = 원형)'],
        ['접시 중심 검출', 'ellipse + rim depth + 보정', 'ellipse fitting만으로 충분'],
        ['depth 정확도', 'rim depth 25th percentile', 'center depth도 OK'],
        ['캘리브레이션 코드', 'step3 그대로', 'step3 그대로'],
    ]),
])

# ──────────────────────────────────────────────────────────────
# Section 6: 재캘리브레이션 조건
# ──────────────────────────────────────────────────────────────
print('섹션 6: 재캘리브레이션 조건...')

append_blocks(cal_page_id, [
    heading(2, '재캘리브레이션이 필요한 경우'),
    bullet('카메라 위치/각도가 변경됐을 때'),
    bullet('로봇 베이스 위치가 바뀌었을 때'),
    bullet('정확도가 10mm 이상으로 나빠졌을 때'),
    bullet_rich(
        text('컨베이어 위치 변경은 재캘리브레이션 불필요', bold=True),
        text(' (카메라-로봇 관계는 동일)')
    ),
])

# ──────────────────────────────────────────────────────────────
# Section 7: AMR 진동 대응
# ──────────────────────────────────────────────────────────────
print('섹션 7: AMR 진동 대응...')

append_blocks(cal_page_id, [
    divider(),
    heading(2, 'AMR 진동 대응'),
    callout('AMR에서 로봇팔 움직일 때 진동이 카메라에 영향 → 촬영 타이밍이 핵심', '⚠️'),
    paragraph(
        text('카메라와 로봇 베이스가 같은 AMR 본체에 고정 → '),
        text('캘리브레이션 행렬(정적 관계)은 유효', bold=True),
    ),
    heading(3, '진동 영향'),
    table(3, [
        ['상태', '영향', '대응'],
        ['팔 정지 시', '진동 없음 → 정상 촬영', '이 타이밍에 촬영 (권장)'],
        ['팔 이동 중 (약한 진동)', '+2~5mm 오차 → 흡착 그리퍼 허용 범위', '짧은 노출 + 멀티프레임 평균'],
        ['팔 이동 중 (심한 진동)', '+5~20mm+ → 검출 실패 가능', '반드시 정지 후 촬영'],
    ]),
    heading(3, '권장 운영 흐름'),
])

operation_flow = """컨베이어 정지
  → 로봇팔 정지 상태에서 촬영 (진동 없음)
  → YOLO seg: 접시 전체 검출 → 좌표 리스트 확정
  → 순차 pick 시작 (좌표는 이미 확정, 이후 카메라 흔들려도 무관)
  → (선택) 다음 접시 전 팔 잠깐 정지 → 재촬영으로 좌표 갱신"""

append_blocks(cal_page_id, [
    code_block(operation_flow),
    heading(3, '진동 심할 때 (2m에서 보일 정도)'),
    numbered('정지 후 촬영이 필수 (0.3~0.5초 대기)'),
    numbered('방진 마운트 추가 검토 (카메라에 방진 고무/댐퍼)'),
    numbered('진동 줄어들면 실시간 전환 가능'),
    callout('핵심: 접시가 멈추고 → 팔 정지 시 촬영 → 좌표 확정 → 이후 접시는 안 움직이므로 pick 정확도 영향 없음', '💡'),
])

# ──────────────────────────────────────────────────────────────
# Section 8: 카메라 배치 방식 비교
# ──────────────────────────────────────────────────────────────
print('섹션 8: 카메라 배치 비교...')

append_blocks(cal_page_id, [
    divider(),
    heading(2, '카메라 배치 방식 비교'),
    table(4, [
        ['방식', '설명', '장점', '단점'],
        ['고정 카메라 1대 (추천)', 'Eye-to-Hand, Top-Down 설치', '가장 빠름, 심플, 캘리브레이션 간단', '진동 시 촬영 타이밍 주의'],
        ['고정 + 팔 카메라', 'Eye-to-Hand + Eye-in-Hand', '최고 정확도 (coarse-to-fine)', '느림, 캘리브레이션 3중, 복잡'],
        ['팔 카메라만', 'Eye-in-Hand only', '진동 무관 (상대 오차 0)', '전체 스캔 불가, 가장 느림'],
    ]),
    callout('결론: 고정 카메라 1대 + Top-Down 설치가 현재 요구사항(속도 최우선)에 최적', '✅'),
])

# ──────────────────────────────────────────────────────────────
# Section 9: 정확도 기준
# ──────────────────────────────────────────────────────────────
print('섹션 9: 정확도 기준...')

append_blocks(cal_page_id, [
    divider(),
    heading(2, '캘리브레이션 정확도 기준'),
    table(3, [
        ['등급', '오차 범위', '적합 용도'],
        ['우수', '1~3mm', '정밀 조립'],
        ['양호', '3~5mm', '일반 pick & place ← 목표'],
        ['보통', '5~10mm', '큰 물체 ← 현재 달성 (avg 5.3mm)'],
        ['불량', '10mm+', '재캘리브레이션 필요'],
    ]),
    paragraph(
        text('현재 결과: ', bold=True),
        text('7포인트 수집, avg 5.3mm, max 7.8mm (2026-03-18)'),
    ),
    paragraph(
        text('Top-Down 설치 시 오프셋 보정 불필요 → 3~5mm 달성 가능', color='blue', bold=True),
    ),
])

print('\n✅ 완료! "로봇 1차 calibration 참조" 페이지 생성됨')
print(f'페이지 ID: {cal_page_id}')
