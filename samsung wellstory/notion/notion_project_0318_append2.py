"""
Notion - 2026-03-18 페이지에 추가 내용 (10~13번)
- 시스템 아키텍처 설계 (AMR + Fixed Camera)
- 로봇 1차 calibration 참조 페이지 생성
- step5_test.py Start→Bowl→Start 루프 구현
- 로봇 API 시행착오 및 해결
"""
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

# ── 10. 양산 시스템 아키텍처 설계 ──
print('  10. 양산 시스템 아키텍처 설계 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '10. 양산 시스템 아키텍처 설계'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '설계 완료'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': '컨베이어 위 접시 10~20개를 듀얼 암 AMR 로봇이 pick & place하는 시나리오에 대한 카메라/캘리브레이션 전략 설계.'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '최종 결정 사항'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '카메라: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Fixed RealSense 1대 (Top-Down 설치 권장)'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '캘리브레이션: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Eye-to-Hand x2 (같은 카메라, 로봇팔 2개 각각 캘리브레이션)'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '촬영 전략: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '"정지 후 촬영" — 컨베이어 정지 → 1프레임 캡처 → 전체 접시 좌표 획득 → 순차 pick'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'AMR 진동 대응'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'AMR은 로봇팔 움직임 시 상당한 진동 발생 (2m 거리 영상에서도 확인)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '카메라와 로봇이 같은 섀시 → 캘리브레이션 행렬은 유효 (상대 위치 불변)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '진동 중 촬영 시 모션블러/depth 노이즈 → "정지 후 촬영"으로 해결'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '실시간 detecting은 진동 안정화(0.5~1초 대기) 필요 → 우선은 정지 후 촬영 방식 채택'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Top-Down vs 비스듬한 설치'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Top-Down: 접시 중심 검출 정확 (편향 없음), depth 정확, 오프셋 보정 불필요'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '비스듬(55°): seg 마스크 편향 발생, 오프셋 보정 필요 (현재 X=-20, Y=-20mm)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '양산 시 Top-Down 설치 강력 권장'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 11. Notion 캘리브레이션 참조 페이지 생성 ──
print('  11. 캘리브레이션 참조 페이지 기록 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '11. Notion "로봇 1차 calibration 참조" 페이지 생성'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RealMan 페이지 하위에 캘리브레이션 참조 문서 생성 완료.'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '위치: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'RealMan > RealSense + 로봇팔 객체인식 Pick & Place 단계별 확인 > 로봇 1차 calibration 참조'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '내용: 시스템 구성도, Eye-to-Hand x2 설정, 캘리브레이션 절차, 좌표 변환 흐름, 재캘리브레이션 조건, AMR 진동 대응, 정확도 기준'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '페이지 ID: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '32715435-1a92-81eb-a6c8-c76d9c07e7b6'}},
            ]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 12. step5_test.py Start→Bowl→Start 루프 구현 ──
print('  12. Start→Bowl→Start 루프 구현 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '12. step5_test.py Start→Bowl→Start 테스트 루프'}}]}},
        {'object': 'block', 'type': 'paragraph', 'paragraph': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '상태: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '구현 완료 + 동작 확인'}, 'annotations': {'color': 'green', 'bold': True}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '기능 설명'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 's키: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Start Point 저장 (현재 관절각을 Get_Current_Arm_State()로 읽어서 저장)'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 'g키: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Start→접시 위(SAFE_HEIGHT)→Start 자동 왕복 테스트'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '동작 흐름 (3단계)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Start Point로 이동 (Movej_Cmd — 관절각 기반, 안전한 이동)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 위 200mm로 이동 (Movej_P_Cmd — 카메라 좌표→로봇 좌표 변환 위치)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '바로 Start Point로 복귀 (Movej_Cmd — 대기 없이 즉시 복귀)'}}]}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '이동 방식 정리'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 'Movej_Cmd(joints, v, r): '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '관절각 직접 지정 → Start Point 등 사전 정의 위치용'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 'Movej_P_Cmd(pose, v, r): '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Cartesian 좌표(xyz+rxryrz) 지정, 관절 공간으로 이동 → 카메라 좌표 변환 위치용'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': 'Movel (미사용): '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '직선 이동 → RM65에서 관절 한계 걸림 빈번, 사용 안 함'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '테스트 결과'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 위치: X=-380.0, Y=3.5, Z=254.6mm → 이동 성공'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'TCP 오차: 목표 vs 실제 0.0mm (Movej_P_Cmd 정확도 확인)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'RM65 작업 범위: X=-434mm + Z=263mm → 리치 한계 초과 (약 610mm), X=-380mm 이내는 안전'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 13. 로봇 API 시행착오 및 해결 ──
print('  13. 로봇 API 시행착오 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '13. 로봇 API 시행착오 및 해결'}}]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '① Movej_Cmd r 파라미터 누락'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '에러: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': "TypeError: Movej_Cmd() missing required positional argument: 'r'"}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '해결: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'r=0 추가 (교시점 사이 블렌딩 반경, 0=정확 정지)'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '② Get_Joint_Degree IndexError'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '에러: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'IndexError: invalid index (robotic_arm.py line 2190)'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '원인: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Get_Joint_Degree()는 7개 float 반환, Movej_Cmd는 self.code(RM65=6) 기준'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '해결: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'Get_Current_Arm_State() 사용 → self.code 기준으로 올바르게 반환'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '③ CONTROLLER_DATA_LOSE_ERR (무시 가능)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '현상: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': "Get_Current_Arm_State() ret[0]이 '9: CONTROLLER_DATA_LOSE_ERR' 문자열 반환"}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '실제: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'ret[1](관절각), ret[2](TCP) 데이터는 정상 — 에러코드만 무시하면 됨'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '해결: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': "조건을 ret[0]==0 → len(ret)>=3 and ret[1] 로 변경"}},
            ]}},
    ]
}, method='PATCH')

api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '④ 작업 범위 초과 (CONTROLLER_DATA_RETURN_FALSE)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '현상: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'X=-434mm, Z=263mm 위치로 Movej_P_Cmd 실패'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '원인: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'RM65 최대 리치 ~610mm, 직선 거리 = √(434²+263²) ≈ 507mm이나 관절 구성상 불가'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '해결: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '접시를 로봇에 더 가깝게 배치 (X=-380mm → 성공)'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '⑤ input() 크래시 → cv2.waitKey() → 대기 제거'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '현상: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': '접시 위 이동 성공 후 input("Enter...") 누르면 프로그램 종료'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '원인: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'input()이 메인 스레드 블로킹 → OpenCV 이벤트 루프 정지 → 로봇 소켓 타임아웃'}},
            ]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [
                {'type': 'text', 'text': {'content': '해결: '}, 'annotations': {'bold': True}},
                {'type': 'text', 'text': {'content': 'input() → cv2.waitKey(0) → 최종적으로 대기 없이 즉시 복귀로 변경 (그리퍼 미장착 상태)'}},
            ]}},

        {'object': 'block', 'type': 'heading_3', 'heading_3': {
            'rich_text': [{'type': 'text', 'text': {'content': '현재 step5_test.py 설정'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'V_MOVE = 20 (Movej_P_Cmd, Movej_Cmd 공통 속도)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'SAFE_HEIGHT = 200mm (접시 위 안전 높이)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'block=True (이동 완료까지 대기 → sleep 불필요)'}}]}},
        {'object': 'block', 'type': 'bulleted_list_item', 'bulleted_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': 'Start→Bowl→Start 3단계 연속 실행 (대기 없음)'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

# ── 14. 다음 단계 ──
print('  14. 다음 단계 작성 중...')
api(f'https://api.notion.com/v1/blocks/{day18_id}/children', {
    'children': [
        {'object': 'block', 'type': 'heading_1', 'heading_1': {
            'rich_text': [{'type': 'text', 'text': {'content': '14. 다음 단계'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '다양한 접시 위치에서 Start→Bowl→Start 반복 테스트 → 오프셋 미세 조정'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '그리퍼(공압 흡착식) 장착 후 실제 pick & place 테스트'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '접시 하강 동작 추가 (SAFE_HEIGHT → 접시 표면 접근)'}}]}},
        {'object': 'block', 'type': 'numbered_list_item', 'numbered_list_item': {
            'rich_text': [{'type': 'text', 'text': {'content': '(선택) 카메라를 Top-Down으로 재설치 → 접시 중심 편향 근본 해결'}}]}},
        {'object': 'block', 'type': 'divider', 'divider': {}},
    ]
}, method='PATCH')

print('\n추가 완료!')
print('2026-03-18 페이지에 10~14번 항목 추가됨')
print('  10. 양산 시스템 아키텍처 설계')
print('  11. Notion 캘리브레이션 참조 페이지 생성')
print('  12. Start→Bowl→Start 테스트 루프 구현')
print('  13. 로봇 API 시행착오 및 해결')
print('  14. 다음 단계')
