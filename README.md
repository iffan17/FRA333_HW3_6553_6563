# FRA333_HW3_6553_6563
## Objective

อธิบายการประยุกต์ใช้องค์ความรู้เกี่ยวกับจลนศาสตร์เชิงอนุพันธ์ (Differential Kinematics) เพื่อควบคุมหุ่นยนต์แขนกลที่มี 3 แกน (RRR Manipulator) โดยใช้สมการ Forward Kinematics และ Jacobian ตามที่กำหนดในโจทย์

## Function Details
### 1.  endEffectorJacobianHW3

**Function** 

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

**Code**

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

**Function** 

    checkSingularityHW3(q:list[float])->bool

**Used for**

ตรวจสอบว่าหุ่นยนต์อยู่ในสภาวะ singularity หรือไม่ โดยสภาวะ singularity เกิดขึ้นเมื่อ Jacobian ไม่สามารถทำงานได้อย่างสมบูรณ์ (ค่า determinant ของ Jacobian เชิงเส้นเป็นศูนย์)

**Step by step**

1. เรียกใช้ฟังก์ชัน `endEffectorJacobianHW3` เพื่อคำนวณ Jacobian เชิงเส้น `(J_v)`
2. คำนวณค่า determinant ของ `J_v`
3. เปรียบเทียบค่า determinant กับค่าความคลาดเคลื่อนที่กำหนด (epsilon = 0.001)

**Result**

ถ้าค่า determinant น้อยกว่า 0.001 ฟังก์ชันจะคืนค่า `True` เพื่อบอกว่าอยู่ในสภาวะ singularity หากไม่ใช่จะคืนค่า `False`

**Code**

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

**Code**

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

### 1.  Test 1

**How to** 

นำโปรแกรมที่เทียบไว้ มาเทียบกับฟังก์ชัน Jacobian ของ robotic toolbox
ในส่วนนี้เป็นการพิสูจน์ผลลัพธ์ที่ได้จากฟังก์ชัน Jacobian ของโปรแกรมที่เขียนขึ้นมา โดยการเปรียบเทียบกับผลลัพธ์ของฟังก์ชัน `jacob0()` ที่มีอยู่ใน robotic toolbox ซึ่งใช้หลักการเดียวกันในการคำนวณ Jacobian matrix สำหรับหุ่นยนต์แขนกลแบบ RRR (Revolute-Revolute-Revolute) โดยใช้ DH Parameter ของหุ่นยนต์ UR5 ที่กำหนดไว้ในโจทย์

**Step by step**

1. กำหนดตัวแปรที่เกี่ยวข้องจากค่าคงที่ของ UR5 เช่น ความยาวของแขนแต่ละแกน `(a_2, a_3)`, ระยะทางแนวดิ่ง `(d_1, d_4)` เป็นต้น
2. สร้างเมทริกซ์สำหรับตำแหน่งปลายหุ่นยนต์ (`endEff`) ที่เป็นผลลัพธ์จากการใช้งานกับเครื่องมือ SE3
3. สร้างโมเดลของหุ่นยนต์ UR5 โดยใช้ DH Parameter ที่กำหนด
4. ใช้ฟังก์ชัน `jacob0()` จาก robotic toolbox เพื่อคำนวณ Jacobian matrix
5. นำผลลัพธ์ที่ได้มาเปรียบเทียบกับผลลัพธ์ที่ได้จากฟังก์ชัน Jacobian ที่เขียนขึ้นเองเพื่อดูความถูกต้อง


จากตัวอย่างโค้ด เรากำหนดค่า q เป็น `[0, -3.14, -0.2]` ซึ่งเป็นมุมของแต่ละข้อต่อ จากนั้นนำผลลัพธ์ของ Jacobian matrix มาทำการเปรียบเทียบ

**Code**

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
    def TestJacobianHW3(q:list[float])->list[float]:
    
        J_e = rbot.jacob0(q) #คำนวณหา jacobian matrix โดยใช้ jacob0
        return J_e

**Result**

 - 2 cases

![](https://cdn.discordapp.com/attachments/999479443745669173/1295054498145763348/image.png?ex=670d40fd&is=670bef7d&hm=18267566d6b3c46e6e678ad0925b6841bf8c63f8eb4421012babcadd34738b63&=)
![](https://cdn.discordapp.com/attachments/999479443745669173/1295054498431111198/image.png?ex=670d40fd&is=670bef7d&hm=e18ad75c51bcd934b877fdc628844482163c76f479c341427dc4e889c95db13e&=)

### 2.  Test 2

**How to** 

ทดลองใช้จุด Singularity ของหุ่นยนต์ UR5 ว่าสามารถตรวจจับได้หรือไม่
ซึ่งในส่วนนี้เป็นการทดสอบความสามารถของฟังก์ชัน `checkSingularityHW3()` ในการตรวจจับจุด singularity ของหุ่นยนต์ UR5 โดยใช้มุมข้อต่อในลักษณะต่างๆ ที่อาจเกิดปัญหา singularity ขึ้น ซึ่งเกิดจากการที่แกนของข้อต่อหลายๆ ข้อตรงกัน (เช่น ข้อศอกและหัวไหล่) ส่งผลให้ไม่สามารถคำนวณการเคลื่อนที่ในทิศทางที่ต้องการได้

**Step by step**

 - ทดสอบด้วยค่ามุมของข้อต่อในสถานการณ์ปกติที่ไม่มี singularity เพื่อให้แน่ใจว่าฟังก์ชันสามารถตรวจจับได้ว่าหุ่นยนต์ไม่ได้อยู่ในสถานการณ์ singularity (`Normal Config`)
 - ทดสอบค่ามุมที่มีความเป็นไปได้ที่จะเกิด singularity ขึ้น เช่น
    - กรณีข้อศอกชี้ลง (Elbow Down Config) ซึ่งเป็นลักษณะของข้อต่อที่อาจทำให้เกิดปัญหา
    - กรณีที่หัวไหล่และข้อศอกอยู่ในแนวเดียวกัน (Shoulder and Elbow Joint Co-linearity) ซึ่งเป็นอีกสถานการณ์ที่อาจทำให้เกิด singularity
 - แสดงผลลัพธ์เพื่อดูว่าฟังก์ชันสามารถตรวจจับจุด singularity ได้หรือไม่

**Expected results**
 - `Normal Config` ควรแสดงค่าเป็น False เนื่องจากไม่ใช่ singularity
 - `Elbow Down Config` และ `Shoulder and Elbow Joint Co-linearity` ควรแสดงค่าเป็น True เนื่องจากเป็นจุดที่อาจเกิด singularity 

**Code**

    def test_2():
            print(f"\nNormal Config  : {checkSingularityHW3([0,10,0])}")
            # test with UR5 possible singularity Config.
            # in this case : Elbow Down, Shoulder and Elbow Joint Co-linearity
            # with extra -0.2 on q3
            print(f"Elbow Down Config : {checkSingularityHW3([3.14,-3.14/2,-0.2])}")
            print(f"Shoulder and Elbow Joint Co-linearity : {checkSingularityHW3([0,-3.14/2,-0.2])}")
            pass


**Result**

![](https://cdn.discordapp.com/attachments/999479443745669173/1295054727511277642/image.png?ex=670d4133&is=670befb3&hm=08f26ca11f9826540043fb71fc964f49da2ee0dc546a67df04877d1de03d981e&=)


### 3. Test 3

**How to**

ทดลองคำนวณแรงบิดที่ข้อต่อโดยใช้ Jacobian Matrix ของหุ่นยนต์ UR5
ในส่วนนี้เป็นการทดสอบความสามารถของฟังก์ชัน `computeEffortHW3()` ที่ใช้ Jacobian matrix ของหุ่นยนต์ในการคำนวณแรงบิดที่เกิดขึ้นจากแรงที่กระทำกับหุ่นยนต์ โดยในที่นี้จะเปรียบเทียบกับผลลัพธ์ที่ได้จากการคำนวณด้วย Robotic Toolbox (`rbot.jacob0()`)

**Step by step**

1. หาค่า Jacobian ของหุ่นยนต์ UR5 ที่กำหนดไว้โดยใช้ Robotic Toolbox
2. นำค่ากำลังที่กระทำกับหุ่นยนต์ในรูปของเวกเตอร์แรงมากระทำที่ส่วนล่างของหุ่นยนต์ (`force_vector`)
3. คำนวณแรงบิดที่ข้อต่อด้วยสมการ `\( \tau = J_v^T \times \text{force\_vector} \)`
4. เปรียบเทียบผลลัพธ์ที่ได้จากฟังก์ชัน `computeEffortHW3()` กับ Robotic Toolbox

**Expected results**

- ผลลัพธ์ที่ได้จากฟังก์ชันควรใกล้เคียงกับการคำนวณโดยใช้ Robotic Toolbox

**Code**

    def test_3(q, w):
    # หา Jacobian ด้วย Robotic Toolsbox
    J = rbot.jacob0(q) 
    J_v = J[:3][:3]

    # get force vector from w matrix (lower half of w)
    w = np.array(w).reshape(6,1) # input is 1x6
    force_vector = w[3:, :]
    
    # compute : tau = J_v^T * force_vector
    tau = np.dot(J_v.T, force_vector)
    return tau[:,0].tolist()

**Result**

![](https://cdn.discordapp.com/attachments/999479443745669173/1295057541721755679/image.png?ex=670d43d2&is=670bf252&hm=91c2af1c1b2bd50ad9efc0fb7286a26a967fa78d4be135b26ebdc4a5d733c353&=)
![](https://cdn.discordapp.com/attachments/999479443745669173/1295060067174842489/image.png?ex=670d462d&is=670bf4ad&hm=48fc712971f4c9b30775e274ca8dfb4b37d63979df0b4b81b4607583a80e89aa&=)

### 4. Testcase print

    q = [0,0,0]
    w = [0,1,0,0,0,1]
    #w = [12,23,34,45,56,67]
    # q = [0.0,-3.14/2,-0.2]
    print(f'q : {q}')
    print(f'Answer 1 : {endEffectorJacobianHW3(q)}')
    print(f'Test 1 : {test_1(q)}')
    print('\nTest 2')
    test_2()
    print('\nTest 3')
    print(f'q : {q}\nw : {w}')
    print(f'Answer 3 : {computeEffortHW3(q, w)}')
    print(f'Test 3 : {test_3(q, w)}')

## Summary

จากผลการทดสอบที่ได้ จะเห็นได้ว่า 

- Test 1 : output ของโปรแกรมที่ออกแบบ ตรงกับ ฟังก์ชัน Jacobian ของ robotic toolsbox 
- Test 2 : ผลลัพท์ Singularity เป็นไปตามที่คาดการณ์ และตรงกับความเป็นจริงของหุ่นต์ UR5
- Test 3 : output ของโปรแกรมที่ออกแบบ ตรงกับ ฟังก์ชัน Jacobian ของ robotic toolsbox

จึงสรุปว่าโปรแกรมทั้ง 3 ข้อมีความถูกต้อง

## Members
- Siriprapha Uppapark 65340500053
- Iffarn Akrameen 65340500063


