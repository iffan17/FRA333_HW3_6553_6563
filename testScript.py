# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ศิริประภา_6553
2.อิฟฟาน_6563
'''
import numpy as np
from FRA333_HW3_6553_6563 import endEffectorJacobianHW3 ,checkSingularityHW3 ,computeEffortHW3
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#


def get_input():
    q1 = float(input("q1: "))  # Joint 1 angle
    q2 = float(input("q2: "))  # Joint 2 angle
    q3 = float(input("q3: "))  # Joint 3 angle
    Mx = float(input("Mx: "))  
    My = float(input("My: "))  
    Mz = float(input("Mz: "))  
    Fx = float(input("Fx: "))  
    Fy = float(input("Fy: "))  
    Fz = float(input("Fz: "))  
    return [[q1, q2, q3], [Mx, My, Mz, Fx, Fy, Fz]]

def test_HW3():
    # Get user input for joint angles and wrench
    q = get_input()[0]  # Obtain joint configuration from the user
    w = get_wrench()[1]  # Obtain wrench components from the user

    # Call the functions to compute the Jacobian, singularity check, and joint efforts
    J_e = np.array(endEffectorJacobianHW3(q))  # Compute the 6x3 Jacobian matrix for the given joint angles
    singularity_flag = checkSingularityHW3(q)  # Check if the given configuration is a singularity
    tau = computeEffortHW3(q, w)  # Compute the required joint efforts (torques) to achieve the given wrench

    # Display the results to the user
    print(f'\nJoint Configuration (q): {q}')
    print(f'Wrench (w): {w}\n')

    print('--------ข้อ 1: Jacobian--------')
    print_matrix(J_e, "Jacobian Matrix (6x3)")  # Print the full Jacobian matrix
    print_matrix(J_e[:3, :3], "Reduced Jacobian (3x3)")  # Print the reduced 3x3 Jacobian matrix (used for singularity check)

    print('--------ข้อ 2: Singularity Check--------')
    # Report whether the configuration is singular (True/False)
    print(f'Singularity: {"Yes" if singularity_flag else "No"}\n')

    print('--------ข้อ 3: Joint Effort--------')
    # Display the computed joint efforts (torques) required to generate the given wrench
    print(f'Joint Efforts (tau): {np.array(tau).flatten()}\n')

#=============================================<การรันฟังก์ชันหลัก>==============================================================#
# Run the main test function when this script is executed
# if __name__ == "__main__":
#     """
#     This block ensures that when the script is run directly, the test_HW3() function is called.
#     It prompts the user for input, computes the necessary kinematic properties, and displays the results.
#     """
#     test_HW3()
test_HW3()