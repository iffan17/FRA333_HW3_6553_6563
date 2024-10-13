# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
import numpy as np
import roboticstoolbox as rtb

from math import pi
from spatialmath import SE3
from HW3_utils import FKHW3
from FRA333_HW3_6553_6563 import endEffectorJacobianHW3 ,checkSingularityHW3 ,computeEffortHW3


#===========================================<ตรวจคำตอบข้อ 1>====================================================#

## วิธีพิสูจน์ : นำโปรแกรมที่เทียบไว้ มาเทียบกับฟังก์ชัน Jacobian ของ roboic toolsbox ##
# กำหนดตัวแปรที่จะใช้งานโดย นำมาจาก config ของ UR5 Robot
d_1 = 0.0892;a_2 = -0.425;a_3 = -0.39243;d_4 = 0.109;d_5 = 0.093;d_6 = 0.082
c_neg90 = 0;s_neg90 = -1

# กำหนด 
endEff = np.array([[c_neg90, 0, s_neg90, a_3 - d_6],
                 [0, 1, 0, -d_5],
                 [-s_neg90, 0, c_neg90, d_4],
                 [0, 0, 0, 1]])
endEffSE = SE3(endEff)

# กำหนดโมเดลหุ่นยนต์โดยใช้ DH Parameter
rbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a= 0, d= d_1, offset=pi),
        rtb.RevoluteMDH(a= 0, alpha= pi/2),
        rtb.RevoluteMDH(a= a_2)
    ],
    tool = endEffSE,
    name = "RRR_Rbot"
)
# เรียกใช้ฟังก์ชัน Jacobian ของ Robotic Toolsbox
def test_1(q:list[float])->list[float]:

    J_e = rbot.jacob0(q) #คำนวณหา jacobian matrix โดยใช้ jacob0
    return J_e
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#

## วิธีพิสูจน์ : ทดลองใช้จุด Singularity ของหุ่นยนต์ UR5 ว่าสามารถตรวจจับได้ไหม##
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
def test_3():
    q = get_input()[0]
    effort = computeEffortHW3(q,w)
#code here

#==============================================================================================================#

q = [0,-3.14,-0.2]
# q = [0.0,-3.14/2,-0.2]
print(f'q : {q}')
print(f'Answer 1 : {endEffectorJacobianHW3(q)}')
print(f'Test 1 : {test_1(q)}')
print('\nTest 2')
test_2()

print('\nTest 3')
