"""현재 로봇 TCP 위치 확인"""
import sys
sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

robot = Arm(RM65, "192.168.1.18")
ret = robot.Get_Current_Arm_State(retry=1)

tcp = ret[2]  # [x, y, z, rx, ry, rz] (미터)
print(f"현재 TCP 위치: ({tcp[0]*1000:.1f}, {tcp[1]*1000:.1f}, {tcp[2]*1000:.1f}) mm")
print(f"현재 TCP 자세: rx={tcp[3]:.3f} ry={tcp[4]:.3f} rz={tcp[5]:.3f}")
print(f"관절 각도: {[f'{j:.1f}' for j in ret[1]]}")

robot.Arm_Socket_Close()
