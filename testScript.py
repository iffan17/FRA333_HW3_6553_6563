# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
import numpy as np
import roboticstoolbox as rtb

from spatialmath import SE3
from HW3_utils import FKHW3
from FRA333_HW3_6553_6563 import endEffectorJacobianHW3 ,checkSingularityHW3 ,computeEffortHW3
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
c_minus_90 = 0
s_minus_90 = -1

# หา End effector และสร้าง rbot
# คำนวณตำแหน่งและการหมุนของ end-effector
End_P = np.array([a_3 + (-d_6), -d_5 , d_4]) # ตำแหน่งของ end-effector
End_R = np.array([[c_minus_90,      0,  s_minus_90], # เมทริกซ์การหมุน
                  [0,               1,           0],
                  [-(s_minus_90),   0,  c_minus_90]])
End = np.eye(4)  # สร้างเมทริกซ์ขนาด 4x4
End[0:3, 3] = End_P # กำหนดส่วนของตำแหน่งในเมทริกซ์
End[0:3, 0:3] = End_R # กำหนดส่วนของการหมุนในเมทริกซ์
End_EF = SE3(End) # กำหนด end effector ในรูปแบบ SE3 (การแปลงHomogeneous)
# print('End_EF is \n', End_EF)
# กำหนดโมเดลหุ่นยนต์โดยใช้ DH Parameter
rbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a= 0, d= d_1, offset=3.14),
        rtb.RevoluteMDH(a= 0, alpha= 3.14/2),
        rtb.RevoluteMDH(a= a_2)
    ],
    tool = End_EF,
    name = "RRR_Rbot"
)
def TestJacobianHW3(q:list[float])->list[float]:
    print('################# ตรวจคำตอบข้อ 1 #################')

    J_e = rbot.jacob0(q) #คำนวณหา jacobian matrix โดยใช้ jacob0
    print("This RRR robot Jacobian is: \n",J_e) 

    #print('\n')
    return J_e

#code here
#print(TestJacobianHW3([0,0,0]))

    


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def test_2():
    print(f"\nNormal Config  : {checkSingularityHW3([0,10,0])}")
    # test with UR5 possible singularity Config.
    # in this case : Elbow Down, Shoulder and Elbow Joint Co-linearity
    # with extra -0.2 on q3
    print(f"Elbow Down Config : {checkSingularityHW3([3.14,-3.14/2,-0.2])}")
    print(f"Shoulder and Elbow Joint Co-linearity : {checkSingularityHW3([0,-3.14/2,-0.2])}")
    pass


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
# def test_3():
#     q = get_input()[0]
#     effort = computeEffortHW3(q,w)
# #code here

#==============================================================================================================#
print(test_2())




#def get_input():
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