# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
from HW3_utils import FKHW3
import numpy as np
import sympy as sp
#=============================================<คำตอบข้อ 1>======================================================#
#code here


def endEffectorJacobianHW3(q:list[float])->list[float]:
    
    """
    Steps 
    1. Create empty matrix
    2. Find Jacobian for 1 joint at a time
        given {j, j_v, j_w, r_i, p_i} as components for current joint
        - J_v[i] = (r_i*[0,0,1]) x (p_e - p_i) -> 3x1
        - J_w[i] = r_i                         -> 3x1
    3. Put each joint's jacobian together
        - J = [J_v;J_w]     -> 6x1 matrix
    """
    # Call for Input
    R, P, R_e, p_e = FKHW3(q)

    # Create empty matrix
    N = len(q) #joint count
    J_v = np.zeros((3, N)) # linear  
    J_w = np.zeros((3, N)) # angular

    # Loop for each joint
    for i in range(N):
        # get rotational matrix
        r_i = np.dot(R[:,:,i], [0,0,1]).T 
        
        # get linear vector
        p_i = P[:,i]
        d = p_e - p_i # compute vector from i to e

        # compute & update value 
        J_v[:,i] = np.cross(r_i,d)  #linear J    (3xN)
        J_w[:,i] = r_i              #angular J  (3xN)

    # Merge linear and angular to full Jacobian
    J = np.vstack((J_v,J_w)) 

    return J

#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:

    # Get reduced jacobian form (only J_v)
    J = endEffectorJacobianHW3(q)
    J_v = J[:3][:3]

    # Find det(J_v)
    det_j = np.linalg.det(J_v)
 
    # Compare with epsilon = 0.001
    if abs(det_j) < 0.001 : 
        return True # is singularity
    else :
        return False
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:

    # Get reduced jacobian form (only J_v)
    J = endEffectorJacobianHW3(q)
    J_v = J[:3][:3]

    # get force vector from w matrix (lower half of w)
    w = np.array(w).reshape(6,1) # input is 1x6
    force_vector = w[3:, :]
    
    # compute : tau = J_v^T * force_vector
    tau = np.dot(J_v.T, force_vector)

    return tau[:,0].tolist() #flatten matrix for output

#==============================================================================================================#

print("J_e :")
print(endEffectorJacobianHW3([0,10,0]))
print(f"\n Singularity : {checkSingularityHW3([0,10,0])}")
print("\nEffort (Tau) :")
print(computeEffortHW3([0,10,0],[0,0,0,0,50,0]))


