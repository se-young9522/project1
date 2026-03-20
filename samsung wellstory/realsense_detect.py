import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from collections import defaultdict
from datetime import datetime
import matplotlib.pyplot as plt
import os

# 학습된 모델 로드
model = YOLO(r'C:\Users\ASUS\Desktop\samsung wellstory\runs\segment\bowl_seg_v53\bowl_seg_v53\weights\best.pt')

# ─── 설정 ─────────────────────────────────────────
CONF_THRESHOLD = 0.7       # 70% 미만 무시
DEPTH_MAX      = 1.5       # 1.5m 이상 무시 (현장에서 거리 측정 필요)
TRACK_FRAMES   = 3         # 3프레임 연속 감지된 것만 표시
RESOLUTION     = (1280, 720)  # 해상도 (1280x720)
SAVE_DIR       = r'C:\Users\ASUS\Desktop\samsung wellstory\recordings'
# ────────────────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, RESOLUTION[0], RESOLUTION[1], rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, RESOLUTION[0], RESOLUTION[1], rs.format.z16, 30)
pipeline.start(config)

# depth-color 정렬
align = rs.align(rs.stream.color)

# temporal smoothing: 영역별 감지 횟수 추적
detect_history = defaultdict(int)  # key: grid_cell → value: 연속 감지 횟수
prev_cells = set()

# 녹화 상태
is_recording = False
video_writer = None
record_name = None
conf_log = []        # 프레임별 인식률 기록 (그래프용)
frame_count = 0

def get_grid_cell(cx, cy, cell_size=80):
    """중심점을 그리드 셀로 변환 (근접 위치 통합)"""
    return (cx // cell_size, cy // cell_size)

def start_recording():
    global is_recording, video_writer, record_name, conf_log, frame_count
    record_name = datetime.now().strftime('%Y%m%d_%H%M%S')
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    vid_path = os.path.join(SAVE_DIR, f'detect_{record_name}.mp4')
    video_writer = cv2.VideoWriter(vid_path, fourcc, 30, RESOLUTION)
    conf_log = []
    frame_count = 0
    is_recording = True
    print(f"\n[REC] 녹화 시작: {record_name}")
    print(f"  영상: {vid_path}")

def stop_recording():
    global is_recording, video_writer, record_name, conf_log, frame_count
    if video_writer:
        video_writer.release()
    video_writer = None
    is_recording = False

    # 인식률 그래프 생성
    graph_path = os.path.join(SAVE_DIR, f'graph_{record_name}.png')
    save_graph(conf_log, graph_path)

    # 구간별 분석 리포트 생성
    report_path = os.path.join(SAVE_DIR, f'report_{record_name}.txt')
    save_report(conf_log, report_path)

    print(f"\n[STOP] 녹화 저장 완료: {record_name}")
    print(f"  영상:   detect_{record_name}.mp4")
    print(f"  그래프: graph_{record_name}.png")
    print(f"  리포트: report_{record_name}.txt")
    print(f"  총 {frame_count}프레임, 약 {frame_count/30:.1f}초")
    record_name = None
    conf_log = []
    frame_count = 0

def save_graph(conf_data, save_path):
    """인식률 시계열 그래프 저장"""
    if not conf_data:
        print("  [그래프] 데이터 없음, 건너뜀")
        return

    frames = [d['frame'] for d in conf_data]
    times  = [f / 30.0 for f in frames]  # 프레임 → 초
    avg_confs   = [d['avg_conf'] * 100 for d in conf_data]
    max_confs   = [d['max_conf'] * 100 for d in conf_data]
    bowl_counts = [d['count'] for d in conf_data]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(f'Bowl Detection Report  ({save_path.split(os.sep)[-1].replace("graph_","").replace(".png","")})',
                 fontsize=14, fontweight='bold')

    # 미감지 구간 (count=0) & 90% 미만 구간 표시용
    no_detect = [t for t, c in zip(times, bowl_counts) if c == 0]
    low_conf  = [t for t, a, c in zip(times, avg_confs, bowl_counts) if c > 0 and a < 90]

    # 상단: 인식률
    ax1.plot(times, avg_confs, color='#2196F3', linewidth=1.5, label='Avg Confidence', alpha=0.8)
    ax1.plot(times, max_confs, color='#4CAF50', linewidth=1, label='Max Confidence', alpha=0.5)
    ax1.fill_between(times, avg_confs, alpha=0.15, color='#2196F3')
    # 미감지 구간 빨간 배경
    for t in no_detect:
        ax1.axvspan(t - 0.02, t + 0.02, color='red', alpha=0.15)
    # 90% 미만 구간 노란 배경
    for t in low_conf:
        ax1.axvspan(t - 0.02, t + 0.02, color='#FFC107', alpha=0.2)
    ax1.set_ylabel('Confidence (%)')
    ax1.set_ylim(0, 105)
    ax1.axhline(y=90, color='#FFC107', linestyle='--', alpha=0.6, label='90% line')
    ax1.axhline(y=CONF_THRESHOLD * 100, color='red', linestyle='--', alpha=0.5, label=f'Threshold ({CONF_THRESHOLD:.0%})')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)

    # 하단: 감지된 그릇 수
    ax2.fill_between(times, bowl_counts, alpha=0.3, color='#FF9800')
    ax2.plot(times, bowl_counts, color='#FF9800', linewidth=1.5, label='Bowl Count')
    for t in no_detect:
        ax2.axvspan(t - 0.02, t + 0.02, color='red', alpha=0.15)
    ax2.set_ylabel('Detected Bowls')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylim(0, max(bowl_counts + [1]) + 1)
    ax2.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax2.legend(loc='lower right')
    ax2.grid(True, alpha=0.3)

    # 통계 요약 텍스트
    total = len(conf_data)
    no_detect_cnt = len(no_detect)
    low_conf_cnt = len(low_conf)
    good_cnt = total - no_detect_cnt - low_conf_cnt
    avg_all = np.mean([d['avg_conf'] * 100 for d in conf_data if d['count'] > 0]) if good_cnt + low_conf_cnt > 0 else 0
    summary = (f"Total: {total} frames ({total/30:.1f}s)  |  "
               f"90%+: {good_cnt} ({good_cnt/total*100:.0f}%)  |  "
               f"<90%: {low_conf_cnt} ({low_conf_cnt/total*100:.0f}%)  |  "
               f"No detect: {no_detect_cnt} ({no_detect_cnt/total*100:.0f}%)  |  "
               f"Avg conf: {avg_all:.1f}%")
    fig.text(0.5, 0.01, summary, ha='center', fontsize=9, color='gray',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  [그래프] 저장: {save_path}")
    print(f"  [통계] {summary}")

def find_segments(conf_data, condition_fn):
    """연속된 프레임을 구간으로 묶기"""
    segments = []
    start = None
    for i, d in enumerate(conf_data):
        if condition_fn(d):
            if start is None:
                start = i
        else:
            if start is not None:
                segments.append((start, i - 1))
                start = None
    if start is not None:
        segments.append((start, len(conf_data) - 1))
    return segments

def fmt_time(frame_idx):
    """프레임 인덱스 → 분:초 형식"""
    sec = frame_idx / 30.0
    m, s = divmod(sec, 60)
    return f"{int(m):02d}:{s:05.2f}"

def save_report(conf_data, save_path):
    """구간별 분석 리포트 저장"""
    if not conf_data:
        return

    total = len(conf_data)
    detect_data = [d for d in conf_data if d['count'] > 0]
    avg_all = np.mean([d['avg_conf'] * 100 for d in detect_data]) if detect_data else 0

    no_detect_segs = find_segments(conf_data, lambda d: d['count'] == 0)
    low_conf_segs  = find_segments(conf_data, lambda d: d['count'] > 0 and d['avg_conf'] * 100 < 90)
    good_segs      = find_segments(conf_data, lambda d: d['count'] > 0 and d['avg_conf'] * 100 >= 90)

    no_detect_frames = sum(e - s + 1 for s, e in no_detect_segs)
    low_conf_frames  = sum(e - s + 1 for s, e in low_conf_segs)
    good_frames      = sum(e - s + 1 for s, e in good_segs)

    lines = []
    lines.append("=" * 70)
    lines.append(f"  Bowl Detection Report  ({os.path.basename(save_path).replace('report_','').replace('.txt','')})")
    lines.append("=" * 70)
    lines.append("")
    lines.append(f"  총 프레임:     {total} ({total/30:.1f}초)")
    lines.append(f"  평균 인식률:   {avg_all:.1f}% (감지된 프레임 기준)")
    lines.append(f"  90%+ 구간:     {good_frames}프레임 ({good_frames/total*100:.0f}%)")
    lines.append(f"  90% 미만:      {low_conf_frames}프레임 ({low_conf_frames/total*100:.0f}%)")
    lines.append(f"  미감지:        {no_detect_frames}프레임 ({no_detect_frames/total*100:.0f}%)")
    lines.append("")

    # 미감지 구간
    lines.append("-" * 70)
    lines.append("  [미감지 구간] 접시를 아예 인식하지 못한 구간")
    lines.append("-" * 70)
    lines.append(f"  {'번호':<6}{'시작':>10}{'끝':>10}{'길이(초)':>10}{'프레임수':>10}")
    lines.append(f"  {'----':<6}{'--------':>10}{'--------':>10}{'--------':>10}{'--------':>10}")
    if no_detect_segs:
        for i, (s, e) in enumerate(no_detect_segs):
            dur = (e - s + 1) / 30.0
            lines.append(f"  {i+1:<6}{fmt_time(s):>10}{fmt_time(e):>10}{dur:>9.2f}s{e-s+1:>10}")
        total_dur = no_detect_frames / 30.0
        lines.append(f"  {'합계':<6}{'':>10}{'':>10}{total_dur:>9.2f}s{no_detect_frames:>10}")
    else:
        lines.append("  (없음)")
    lines.append("")

    # 90% 미만 구간
    lines.append("-" * 70)
    lines.append("  [90% 미만 구간] 인식은 되지만 인식률이 낮은 구간")
    lines.append("-" * 70)
    lines.append(f"  {'번호':<6}{'시작':>10}{'끝':>10}{'길이(초)':>10}{'평균':>10}{'최저':>10}")
    lines.append(f"  {'----':<6}{'--------':>10}{'--------':>10}{'--------':>10}{'--------':>10}{'--------':>10}")
    if low_conf_segs:
        for i, (s, e) in enumerate(low_conf_segs):
            dur = (e - s + 1) / 30.0
            seg_data = conf_data[s:e+1]
            seg_avg = np.mean([d['avg_conf'] * 100 for d in seg_data if d['count'] > 0])
            seg_min = min([d['avg_conf'] * 100 for d in seg_data if d['count'] > 0])
            lines.append(f"  {i+1:<6}{fmt_time(s):>10}{fmt_time(e):>10}{dur:>9.2f}s{seg_avg:>9.1f}%{seg_min:>9.1f}%")
        total_dur = low_conf_frames / 30.0
        lines.append(f"  {'합계':<6}{'':>10}{'':>10}{total_dur:>9.2f}s")
    else:
        lines.append("  (없음)")
    lines.append("")

    # 안정적 구간
    lines.append("-" * 70)
    lines.append("  [안정 구간] 90% 이상 인식률로 안정적인 구간")
    lines.append("-" * 70)
    lines.append(f"  {'번호':<6}{'시작':>10}{'끝':>10}{'길이(초)':>10}{'평균':>10}")
    lines.append(f"  {'----':<6}{'--------':>10}{'--------':>10}{'--------':>10}{'--------':>10}")
    if good_segs:
        for i, (s, e) in enumerate(good_segs):
            dur = (e - s + 1) / 30.0
            seg_data = conf_data[s:e+1]
            seg_avg = np.mean([d['avg_conf'] * 100 for d in seg_data if d['count'] > 0])
            lines.append(f"  {i+1:<6}{fmt_time(s):>10}{fmt_time(e):>10}{dur:>9.2f}s{seg_avg:>9.1f}%")
        total_dur = good_frames / 30.0
        lines.append(f"  {'합계':<6}{'':>10}{'':>10}{total_dur:>9.2f}s")
    else:
        lines.append("  (없음)")
    lines.append("")
    lines.append("=" * 70)

    report_text = '\n'.join(lines)
    with open(save_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    print(f"  [리포트] 저장: {save_path}")

print("RealSense 그릇 감지 시작")
print(f"  해상도: {RESOLUTION[0]}x{RESOLUTION[1]}")
print(f"  conf ≥ {CONF_THRESHOLD:.0%} | depth ≤ {DEPTH_MAX}m | 연속 {TRACK_FRAMES}프레임")
print("  w: 녹화 시작 | e: 녹화 저장 | q: 종료")

try:
    while True:
        frames      = pipeline.wait_for_frames()
        aligned     = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        results     = model(color_image, conf=CONF_THRESHOLD, verbose=False)

        current_cells = set()
        detections = []

        for r in results:
            if r.masks is None:
                # detection 모델일 경우 boxes만 사용
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    detections.append((x1, y1, x2, y2, conf, cx, cy, None))
            else:
                # seg 모델: 마스크 + 박스
                for mask, box in zip(r.masks, r.boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    mask_pts = mask.xy[0].astype(np.int32) if len(mask.xy[0]) > 0 else None
                    detections.append((x1, y1, x2, y2, conf, cx, cy, mask_pts))

        # 유효 감지 (깊이+temporal 필터 통과한 것)
        valid_confs = []

        for x1, y1, x2, y2, conf, cx, cy, mask_pts in detections:
            # 깊이 필터링
            depth_m = depth_frame.get_distance(cx, cy)
            if depth_m > DEPTH_MAX and depth_m > 0:
                continue

            # temporal smoothing
            cell = get_grid_cell(cx, cy)
            current_cells.add(cell)
            detect_history[cell] += 1

            if detect_history[cell] < TRACK_FRAMES:
                continue

            valid_confs.append(conf)

            # seg 마스크 오버레이
            if mask_pts is not None and len(mask_pts) > 2:
                overlay = color_image.copy()
                cv2.fillPoly(overlay, [mask_pts], (0, 200, 0))
                color_image = cv2.addWeighted(overlay, 0.3, color_image, 0.7, 0)
                cv2.polylines(color_image, [mask_pts], True, (0, 255, 0), 2)

            # bbox + 정보 표시
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Bowl {conf:.0%}  {depth_m:.2f}m"
            cv2.putText(color_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

        # 사라진 셀의 카운트 리셋
        for cell in list(detect_history.keys()):
            if cell not in current_cells:
                detect_history[cell] = 0

        # 녹화 중이면 프레임 저장 + 인식률 기록
        if is_recording:
            # REC 표시
            cv2.circle(color_image, (30, 30), 12, (0, 0, 255), -1)
            cv2.putText(color_image, "REC", (50, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            video_writer.write(color_image)
            conf_log.append({
                'frame': frame_count,
                'avg_conf': np.mean(valid_confs) if valid_confs else 0,
                'max_conf': max(valid_confs) if valid_confs else 0,
                'count': len(valid_confs),
            })
            frame_count += 1

        cv2.imshow('Bowl Detection - RealSense', color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if is_recording:
                stop_recording()
            break
        elif key == ord('w') and not is_recording:
            start_recording()
        elif key == ord('e') and is_recording:
            stop_recording()

finally:
    if is_recording:
        stop_recording()
    pipeline.stop()
    cv2.destroyAllWindows()
