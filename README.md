# FRA333_HW3_6553_6563
## Objective
อธิบายการประยุกต์ใช้องค์ความรู้เกี่ยวกับจลนศาสตร์เชิงอนุพันธ์ (Differential Kinematics) เพื่อควบคุมหุ่นยนต์แขนกลที่มี 3 แกน (RRR Manipulator) โดยใช้สมการ Forward Kinematics และ Jacobian ตามที่กำหนดในโจทย์
## Function Details
### 1.  endEffectorJacobianHW3
**Function :** 

    endEffectorJacobianHW3(q:list[float])->list[float]

**Used for**
ใช้คำนวณ Jacobian Matrix ของหุ่นยนต์แขนกล โดยสร้างเมทริกซ์ที่ใช้ระบุความสัมพันธ์ระหว่างความเร็วของข้อต่อและความเร็วเชิงเส้นและเชิงมุมของ end-effector (ตำแหน่งปลายแขนกล)
**Step by step**
1. สร้างเมทริกซ์เปล่าสำหรับ Jacobian โดยแยกเป็นส่วนเชิงเส้น `(J_v)` และส่วนเชิงมุม `(J_w)`
2. คำนวณแต่ละส่วนของ Jacobian ทีละข้อ โดยใช้
    -   `J_v = (r_i x (p_e - p_i))` เพื่อระบุตำแหน่งเชิงเส้น
    -   `J_w = r_i` เพื่อระบุตำแหน่งเชิงมุม
3. รวมทั้งสองส่วนเข้าด้วยกันเป็น Jacobian ขนาด 6xN

**Result**
ค่า Jacobian ที่สามารถนำไปใช้ในการคำนวณอื่น ๆ เช่น การทำนายการเคลื่อนที่ของ end-effector

    def endEffectorJacobianHW3(q:list[float])->list[float]:
        # Call for Input
        R, P, R_e, p_e = FKHW3(q)
    
        # Create empty matrix
        N = len(q) #joint count
        J_v = np.zeros((3, N)) # linear  
        J_w = np.zeros((3, N)) # angular
    
        # Loop for each joint
        for i in range(N):
        
        r_i = np.dot(R[:,:,i], [0,0,1]).T   # get rotational matrix
        p_i = P[:,i]                        # get linear vector
        d = p_e - p_i                       # compute vector from i to e

        # compute & update value 
        J_v[:,i] = np.cross(r_i,d)  #linear J       (3xN)
        J_w[:,i] = r_i              #angular J      (3xN)

    # Merge linear and angular to full Jacobian
    J = np.vstack((J_v,J_w)) 

    return J

### 2. checkSingularityHW3
**Function :** 

    checkSingularityHW3(q:list[float])->bool

**Used for**
ตรวจสอบว่าหุ่นยนต์อยู่ในสภาวะ singularity หรือไม่ โดยสภาวะ singularity เกิดขึ้นเมื่อ Jacobian ไม่สามารถทำงานได้อย่างสมบูรณ์ (ค่า determinant ของ Jacobian เชิงเส้นเป็นศูนย์)
**Step by step**
1. เรียกใช้ฟังก์ชัน `endEffectorJacobianHW3` เพื่อคำนวณ Jacobian เชิงเส้น `(J_v)`
2. คำนวณค่า determinant ของ `J_v`
3. เปรียบเทียบค่า determinant กับค่าความคลาดเคลื่อนที่กำหนด (epsilon = 0.001)

**Result**
ถ้าค่า determinant น้อยกว่า 0.001 ฟังก์ชันจะคืนค่า `True` เพื่อบอกว่าอยู่ในสภาวะ singularity หากไม่ใช่จะคืนค่า `False`

    def checkSingularityHW3(q:list[float])->bool:
    
        # Get reduced jacobian form (only J_v)
        J = endEffectorJacobianHW3(q)
        J_v = J[:3][:3]
    
        # Find det(J_v)
        det_j = np.linalg.det(J_v)
        abs(det_j)
        # Compare with epsilon = 0.001
        if abs(det_j) < 0.001 : 
            return True # is singularity
        else :
            return False

### 3.  computeEffortHW3
**Function :** 

    computeEffortHW3(q:list[float], w:list[float])->list[float]

**Used for**
คำนวณแรงบิด (Torque) ที่เกิดขึ้นในข้อต่อของหุ่นยนต์แขนกล โดยใช้ Jacobian เชิงเส้นและแรงภายนอกที่กระทำกับ end-effector
**Step by step**
1. เรียกใช้ฟังก์ชัน `endEffectorJacobianHW3` เพื่อคำนวณ Jacobian เชิงเส้น `(J_v)`
2. นำค่าแรงภายนอก (w) ที่กระทำกับ end-effector มาคำนวณแรงบิดโดยใช้สมการ `\tau = J_v^T \cdot \text{force_vector}` 
- `force_vector` คือแรงที่กระทำต่อ end-effector ในแกนเชิงเส้น

**Result**
ค่าแรงบิดที่กระทำในข้อต่อแต่ละข้อ

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

## Check Answer
### Question 1



### Question 2



### Question 3




## Members
- Siriprapha Uppapark 65340500053
- Iffarn Akrameen 65340500063

