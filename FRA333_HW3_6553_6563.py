# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
from HW3_utils import FKHW3
import numpy as np

#=============================================<คำตอบข้อ 1>======================================================#
#code here


def endEffectorJacobianHW3(q:list[float])->list[float]:
    
    """
    Steps :
    1. Create empty matrix
    2. Find Jacobian for 1 joint at a time
        given {j, j_v, j_w, r_i, p_i} as components for current joint
        - J_v[i] = (r_i*[0,0,1]) x (p_e - p_i) -> 3x1
        - J_w[i] = r_i                         -> 3x1
    3. Put each joint's jacobian together
        - J = [J_v;J_w]     -> 6x1 matrix
    """
    R, P, R_e, p_e = FKHW3(q)
    # Create empty matrix
    
    N = len(q) #joint count
    J_v = np.zeros((3, N)) # linear  
    J_w = np.zeros((3, N)) # angular

    # Loop for each joint
    for i in range(N):
        # get rotational matrix
        r_i = np.dot(R[:,:,i], [0,0,1]).T
        print(f"r_i : {r_i}")
        
        # get linear vector
        p_i = P[:,i]
        d = p_e - p_i # compute vector from i to e

        # update value 
        J_v[:,i] = np.cross(r_i,d)  #linear J   (3xN)
        J_w[:,i] = r_i              #angular J  (3xN)
        print(f"joint : {i}")
        print(J_v[:,i])
        print(J_w[:,i])
    
    J = np.vstack((J_v,J_w)) # Merge linear and angular to full Jacobian
    print(J)
    return "Work done"

#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#
def temp(q:list[float])->list[float]:

    pass
print(endEffectorJacobianHW3([0,10,0]))


