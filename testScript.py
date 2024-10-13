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


def get_joint_configuration():
    """
    Prompt the user to input joint angles (q1, q2, q3) in radians.
    
    In robotic systems, the joint configuration (q) represents the angles of the joints 
    that define the position and orientation of the end effector. Here, we assume a 3-DOF system.
    
    Returns:
    A list of 3 joint angles [q1, q2, q3] in radians.
    """
    print("Please input the joint angles (q1, q2, q3) in radians.")
    q1 = float(input("q1: "))  # Joint 1 angle
    q2 = float(input("q2: "))  # Joint 2 angle
    q3 = float(input("q3: "))  # Joint 3 angle
    return [q1, q2, q3]

# Function to get user input for wrench w (the forces and torques acting on the end effector)
def get_wrench():
    """
    Prompt the user to input the wrench (forces and torques) applied at the end effector.
    
    The wrench is composed of moments (torques) [Mx, My, Mz] and forces [Fx, Fy, Fz]. 
    These values represent the external forces and torques that need to be translated 
    into joint torques through the Jacobian matrix.
    
    Returns:
    A list of 6 values [Mx, My, Mz, Fx, Fy, Fz] representing the wrench.
    """
    print("Please input the wrench (moment and force) components (Mx, My, Mz, Fx, Fy, Fz).")
    Mx = float(input("Mx: "))  # Moment around x-axis
    My = float(input("My: "))  # Moment around y-axis
    Mz = float(input("Mz: "))  # Moment around z-axis
    Fx = float(input("Fx: "))  # Force along x-axis
    Fy = float(input("Fy: "))  # Force along y-axis
    Fz = float(input("Fz: "))  # Force along z-axis
    return [Mx, My, Mz, Fx, Fy, Fz]

# Helper function to format and print matrices nicely
def print_matrix(matrix, name):
    """
    Print the matrix in a formatted way to improve readability.

    Args:
    matrix: The matrix to be printed.
    name: The label or name of the matrix (e.g., 'Jacobian Matrix').
    """
    print(f"{name}:")
    for row in matrix:
        formatted_row = "  ".join([f"{val:10.5f}" for val in row])
        print(f"[{formatted_row}]")
    print()

def test_HW3():
    """
    Main testing function to compute the Jacobian, check for singularity, and compute joint efforts
    for a given set of joint configurations (q) and wrench (w).
    
    This function is designed to interact with the user to get the joint angles and wrench components
    and use the previously implemented functions to calculate the Jacobian matrix, check singularity, and compute efforts.
    
    The test is split into three main sections corresponding to:
    1. Jacobian calculation (ข้อ 1)
    2. Singularity check (ข้อ 2)
    3. Joint effort calculation (ข้อ 3)
    """
    
    # Get user input for joint angles and wrench
    q = get_joint_configuration()  # Obtain joint configuration from the user
    w = get_wrench()  # Obtain wrench components from the user

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