"""
MoveL 동작 테스트
  h: 홈 자세로 이동 (0, 0, 90, 0, 90, 0) - Movej
  z: 현재 위치에서 Z 5cm 아래로 MoveL
  b: 홈 자세로 복귀 (0, 0, 90, 0, 90, 0) - Movej
  q: 종료
"""
import sys
import time

sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

SPEED      = 20
DOWN_MM    = 50   # Z 아래로 내릴 거리 (mm)
HOME_JOINTS = [0, 0, 90, 0, 90, 0]

print("=" * 50)
print("  MoveL 테스트")
print("  h: 홈 자세 이동")
print("  z: Z -5cm MoveL")
print("  b: 홈 자세 복귀")
print("  q: 종료")
print("=" * 50)

robot = Arm(RM65, "192.168.1.18")
print(f"[OK] 연결 (API: {robot.API_Version()})")

while True:
    key = input("\n명령 (h/z/b/q): ").strip().lower()

    if key == 'q':
        break

    elif key == 'h':
        print(f"  [Movej] 홈 자세 이동: {HOME_JOINTS}")
        result = robot.Movej_Cmd(HOME_JOINTS, SPEED, 0)
        print(f"  결과: {result}")
        time.sleep(4)  # 이동 완료 대기
        ret = robot.Get_Current_Arm_State()
        if ret and ret[2]:
            t = ret[2]
            print(f"  TCP: ({t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f})mm")
            print(f"  관절: {[f'{j:.1f}' for j in ret[1]]}")

    elif key == 'z':
        ret = robot.Get_Current_Arm_State()
        if not ret or not ret[2]:
            print("  [!] TCP 읽기 실패")
            continue
        tcp = ret[2]
        target = [tcp[0], tcp[1], tcp[2] - DOWN_MM/1000, tcp[3], tcp[4], tcp[5]]
        print(f"  [MoveL] Z: {tcp[2]*1000:.1f} → {target[2]*1000:.1f}mm  (↓{DOWN_MM}mm)")
        result = robot.Movel_Cmd(target, SPEED, 0, 0, True)
        print(f"  결과: {result}")
        time.sleep(4)  # 이동 완료 대기
        ret2 = robot.Get_Current_Arm_State()
        if ret2 and ret2[2]:
            t2 = ret2[2]
            dz = (t2[2] - tcp[2]) * 1000
            print(f"  실제 TCP: ({t2[0]*1000:.1f}, {t2[1]*1000:.1f}, {t2[2]*1000:.1f})mm")
            print(f"  Z 이동량: {dz:.1f}mm  (목표 -{DOWN_MM}mm)")
            if abs(dz + DOWN_MM) < 5:
                print(f"  [OK] MoveL 정상!")
            else:
                print(f"  [!] 오차 {abs(dz + DOWN_MM):.1f}mm")

    elif key == 'b':
        print(f"  [Movej] 홈 자세 복귀: {HOME_JOINTS}")
        result = robot.Movej_Cmd(HOME_JOINTS, SPEED, 0)
        print(f"  결과: {result}")
        time.sleep(4)  # 이동 완료 대기
        ret = robot.Get_Current_Arm_State()
        if ret and ret[2]:
            t = ret[2]
            print(f"  TCP: ({t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f})mm")
            print(f"  관절: {[f'{j:.1f}' for j in ret[1]]}")

robot.Arm_Socket_Close()
print("\n[OK] 종료")
