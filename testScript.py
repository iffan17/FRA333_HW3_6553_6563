# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
import numpy as np
import roboticstoolbox as rtb

from HW3_utils import FKHW3
from FRA333_HW3_6553_6563 import endEffectorJacobianHW3 ,checkSingularityHW3 ,computeEffortHW3
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def get_input():
    # q1 = float(input("q1: "))  # Joint 1-3 angle
    # q2 = float(input("q2: "))  
    # q3 = float(input("q3: "))  
    # Mx = float(input("Mx: "))  # Moment Vector xyz
    # My = float(input("My: "))  
    # Mz = float(input("Mz: "))  
    # Fx = float(input("Fx: "))  # Force Vector xyz
    # Fy = float(input("Fy: "))  
    # Fz = float(input("Fz: "))  
    #return [[q1, q2, q3], [Mx, My, Mz, Fx, Fy, Fz]]
    [q1, q2, q3] = [0,3.14,2]
    [Mx, My, Mz, Fx, Fy, Fz] = [0,1,0,1,0,1]
    
def test_1():

    #robot = rtb.UR5()

    q = get_input()[0]

    J = endEffectorJacobianHW3(q)
    p_e = FKHW3(q)[-1]
    
    q_check = [q[0] + 0.001,q[1],q[2]]
    p_e_check = FKHW3(q_check)[-1]

    velo_num = (p_e_check - p_e/0.001)
    velo_jaco = J*0.001
    print(velo_num)
    print(velo_jaco)

    return velo_num-velo_jaco

#print(test_1())

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def test_2():
    q = get_input()[0]
    flag = checkSingularityHW3(q)

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
def test_3():
    q = get_input()[0]
    effort = computeEffortHW3(q,w)
#code here

#==============================================================================================================#


robot = rtb.UR5()
q = [0,3.14,2]
w = [0,1,0,1,0,1]
J_ee = robot.jacobe(q)
print(J_ee)