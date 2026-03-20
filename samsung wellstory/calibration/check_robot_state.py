"""로봇 Get_Current_Arm_State 반환값 구조 확인"""
import sys
sys.path.insert(0, r'C:\Users\ASUS\Desktop\samsung wellstory\RM_API-main\Example\API_Example_Python\API_Example_Python')
from robotic_arm_package.robotic_arm import *

robot = Arm(RM65, "192.168.1.18")
print(f"API: {robot.API_Version()}")

ret = robot.Get_Current_Arm_State(retry=1)

print(f"\nret 타입: {type(ret)}")
print(f"ret 길이: {len(ret)}")

for i, item in enumerate(ret):
    print(f"\nret[{i}]: {type(item).__name__} = {item}")

robot.Arm_Socket_Close()
