#!/usr/bin/env python
# coding=utf-8

import math
import cv2 #引用OpenCV功能包
import numpy as np #引用数组功能包

global ARM_Range_False, ARM_Inverse_Mode
global Target_Angle_A_Inverse, Target_Angle_B_Inverse, Target_Angle_C_Inverse
global ARM_Target_X, ARM_Target_Y, ARM_Target_Z
global Judge1, Judge2, Judge3, Judge4, Judge5, Judge6
global Radius1, Radius2, Radius3, Radius4

PI=3.1415926535897
ARM_Range_False=0
ARM_Inverse_Mode=0
ARM_Target_X=0.062
ARM_Target_Y=0.0
ARM_Target_Z=-0.008
Target_Angle_A_Inverse=0
Target_Angle_B_Inverse=0
Target_Angle_C_Inverse=0

AngleB_Max=1.5707963
AngleB_Min=-0.17000
AngleB_Range=AngleB_Max-AngleB_Min
AngleC_Max=0.7273
AngleC_Min=-1.1790
AngleC_Range=AngleC_Max-AngleC_Min

#运动范围判断相关
Judge1=0
Judge2=0
Judge3=0
Judge4=0
Judge5=0
Judge6=0
Radius1=0
Radius2=0
Radius3=0
Radius4=0

B1=0 #Target_Angle_B_Inverse的一部分
B2=0 #Target_Angle_B_Inverse的一部分
R1=0.140 #B杆的长度
R2=0.160 #C杆的长度

####运动范围相关####
#圆弧1的圆心点
Arc_1_x=0
Arc_1_y=R1
#圆弧1的起始角度
Arc_1_start_angle=AngleC_Max
#圆弧1的半径
Arc_1_radius=R2 #运动范围相关

#圆弧2的圆心点
Arc_2_x=0
Arc_2_y=0
#圆弧2的起点位置
PointD_x=0+R2*math.cos(AngleC_Min)
PointD_y=R1+R2*math.sin(AngleC_Min)
#圆弧2的起始角度
Arc_2_start_angle=math.atan(PointD_y/PointD_x)
#圆弧2的半径
Arc_2_radius=pow(PointD_x*PointD_x+PointD_y*PointD_y, 0.5) #运动范围相关

#圆弧3的圆心点
Arc_3_x=R1*math.cos(AngleB_Min)
Arc_3_y=R1*math.sin(AngleB_Min)
#圆弧3的起始角度
Arc_3_start_angle=AngleC_Max+PI/2-AngleB_Range-1.5708
print(Arc_3_start_angle)
#圆弧3的半径
Arc_3_radius=R2 #运动范围相关

#圆弧4的圆心点
Arc_4_x=0
Arc_4_y=0
#圆弧4的起点位置
PointE_x=0+R2*math.cos(AngleC_Max)
PointE_y=R1+R2*math.sin(AngleC_Max)
#圆弧4的起始角度
Arc_4_start_angle=math.atan(PointE_y/PointE_x)
#圆弧4的半径
Arc_4_radius=pow(PointE_x*PointE_x+PointE_y*PointE_y, 0.5) #运动范围相关

arm_base_height=0.3
####运动范围相关####

img_height=500 #画布大小
img_width=700  #画布大小

#二自由度机械臂运动学逆解：先判断目标点是否在运动范围，在则进行运动学逆解
def ARM_Inverse_Solution(x, y, z):
	global ARM_Range_False
	#print("X:"+str(x)+"  Y:"+str(y)+"  Z:"+str(z))

	h=z
	r=pow((x*x+y*y), 0.5) #r为半径，必为正
	Angle=math.acos(x/r)  #θ=asin(y/r);

	#print("h:"+str(h)+"  r:"+str(r)+"  Angle:"+str(Angle))
	
	Target_Angle_A_Inverse=Angle
	
	if(ARM_Range_judge(r, h)==0):
		#print("\nARM_Range_True")	
		ARM_Range_False=0
		ARM_Inverse_Solution_2(r, h) #求出Target_Angle_B、Target_Angle_C
	
	else:
		#print("\nARM_Range_False")
		ARM_Range_False=1	

#运动学逆解方法1	
def ARM_Inverse_Solution_1(x, y):
	global Target_Angle_B_Inverse
	global Target_Angle_C_Inverse
	A=0
	B=0
	C=0
	T=0
	A=2*R1*x
	B=2*R1*y
	C=x*x+y*y+R1*R1-R2*R2
	T=(B+pow(B*B+A*A-C*C,0.5))/(A+C); #T=(B-pow((B*B+A*A-C*C),0.5))/(A+C)

	#print("\nA:"+str(A)+"\nB:"+str(B)+"\nC:"+str(C)+"\nT:"+str(T))
	Target_Angle_B_Inverse=2*math.atan(T)
	Target_Angle_C_Inverse=math.asin((x*math.sin(Target_Angle_B_Inverse)-y*math.cos(Target_Angle_B_Inverse))/R2)
	Target_Angle_B_Inverse=Target_Angle_B_Inverse
	Target_Angle_C_Inverse=Target_Angle_C_Inverse
	Target_Angle_B_Inverse=Target_Angle_B_Inverse*57.3
	Target_Angle_C_Inverse=Target_Angle_C_Inverse*57.3
	#print("\nTarget_Angle_A_Inverse:"+str(Target_Angle_A_Inverse)+"\nTarget_Angle_B_Inverse:"+str(Target_Angle_B_Inverse)+"\nTarget_Angle_C_Inverse:"+str(Target_Angle_C_Inverse))

#运动学逆解方法2
def ARM_Inverse_Solution_2(x,  y):
	global Target_Angle_B_Inverse
	global Target_Angle_C_Inverse
	global R, B1, B2

	R=pow((x*x+y*y), 0.5)

	B1=math.acos((R1*R1+R*R-R2*R2)/(2*R1*R))
	B2=math.atan(y/x)
	Target_Angle_B_Inverse=B1+B2

	Target_Angle_C_Inverse=math.acos((R1*R1+R2*R2-R*R)/(2*R1*R2))

	#print("\nTarget_Angle_A_Inverse:"+str(Target_Angle_A_Inverse)+"\nTarget_Angle_B_Inverse:"+str(Target_Angle_B_Inverse)+"\nTarget_Angle_C_Inverse:"+str(Target_Angle_C_Inverse))
	Target_Angle_B_Inverse=Target_Angle_B_Inverse*57.3
	Target_Angle_C_Inverse=Target_Angle_C_Inverse*57.3
	#print("\nTarget_Angle_A_Inverse:"+str(Target_Angle_A_Inverse)+"\nTarget_Angle_B_Inverse:"+str(Target_Angle_B_Inverse)+"\nTarget_Angle_C_Inverse:"+str(Target_Angle_C_Inverse))

#目标点是否在运动范围判断
def ARM_Range_judge(x, y):
	global Judge1, Judge2, Judge3, Judge4, Judge5, Judge6
	global Radius1, Radius2, Radius3, Radius4

	Radius1 = pow((x-Arc_1_x)*(x-Arc_1_x)+(y-Arc_1_y)*(y-Arc_1_y),0.5)
	Radius2 = pow((x-Arc_2_x)*(x-Arc_2_x)+(y-Arc_2_y)*(y-Arc_2_y),0.5)	
	Radius3 = pow((x-Arc_3_x)*(x-Arc_3_x)+(y-Arc_3_y)*(y-Arc_3_y),0.5)
	Radius4 = pow((x-Arc_4_x)*(x-Arc_4_x)+(y-Arc_4_y)*(y-Arc_4_y),0.5)

	#print("\nRadius1:"+str(Radius1)+"\nRadius2:"+str(Radius2)+"\nRadius3:"+str(Radius3)+"\nRadius4:"+str(Radius4))

	if(Radius1<R2): 
		Judge1=1
	else:
		Judge1=0
	if(Radius2<Arc_2_radius): 
		Judge2=1
	else:
		Judge2=0
	if(Radius3>R2 and y<-0.0811): 
		Judge3=1
	else:
		Judge3=0
	if(Radius4>Arc_4_radius): 
		Judge4=1
	else:
		Judge4=0
	if(y<-arm_base_height): 
		Judge5=1
	else:
		Judge5=0

	if(Judge1==0 and Judge2==0 and Judge3==0 and Judge4==0 and Judge5==0):
		ARM_Inverse_Solution_2(x, y)
		if((Target_Angle_B_Inverse+Target_Angle_C_Inverse)<90):
			Judge6=1
		else:
			Judge6=0

	#print("Judge1:"+str(Judge1)+" Judge2:"+str(Judge2)+" Judge3:"+str(Judge3)+" Judge4:"+str(Judge4))
	if(Judge1 or Judge2 or Judge3 or Judge4 or Judge5 or Judge6):
		return 1
	else:
		return 0

#OpenCV画圆弧
def Draw_half_circle(img, center_x, center_y, radius, start_rad, end_rad, density, thickness):
	origin_x=30
	origin_y=img_height-200

	last_x=center_x+radius*math.cos(end_rad+(2*PI)/density)
	last_y=center_y+radius*math.sin(end_rad+(2*PI)/density)
	for i in range(int(density*abs(end_rad-start_rad)/(2*PI))):
		if(end_rad>start_rad):
			x=center_x+radius*math.cos(start_rad+i*(2*PI)/density)
			y=center_y+radius*math.sin(start_rad+i*(2*PI)/density)
		elif(end_rad<start_rad):
			x=center_x+radius*math.cos(end_rad+i*(2*PI)/density)
			y=center_y+radius*math.sin(end_rad+i*(2*PI)/density)
		cv2.line(img, (int(x)+origin_x, origin_y-int(y)), (int(last_x)+origin_x, origin_y-int(last_y)), (0, 0, 255), thickness)
		last_x=x
		last_y=y

#由于机械结构原因，机械臂的C杆与水平面(X轴)的夹角不能小于-90°而产生的运动范围
def Draw_range_4(img, start_rad, end_rad, density, thickness):
	origin_x=30
	origin_y=img_height-200

	last_x=0.14*math.cos(0*(2*PI)/density+start_rad)*1000
	last_y=0.14*math.sin(0*(2*PI)/density+start_rad)*1000
	AngleB=0*(2*PI)/density+start_rad
	AngleC=PI/2-AngleB
	last_x=last_x+0.16*math.cos(AngleC-PI+AngleB)*1000
	last_y=last_y+0.16*math.sin(AngleC-PI+AngleB)*1000
	
	range_rad=start_rad-end_rad
	for i in range(int(density*abs(range_rad)/(2*PI))):
		x=0.14*math.cos(i*(2*PI)/density+start_rad)*1000
		y=0.14*math.sin(i*(2*PI)/density+start_rad)*1000
		AngleB=i*(2*PI)/density+start_rad
		AngleC=PI/2-AngleB
		x=x+0.16*math.cos(AngleC-PI+AngleB)*1000
		y=y+0.16*math.sin(AngleC-PI+AngleB)*1000
		cv2.line(img, (int(x)+origin_x, origin_y-int(y)), (int(last_x)+origin_x, origin_y-int(last_y)), (0, 255, 255), thickness)
		last_x=x
		last_y=y

#OpenCV画运动范围
def Draw_Range_Judge(img):
	Draw_half_circle(img, Arc_1_x*1000, Arc_1_y*1000, Arc_1_radius*1000, Arc_1_start_angle, Arc_1_start_angle-AngleC_Range, 720, 2)
	Draw_half_circle(img, Arc_2_x*1000, Arc_2_y*1000, Arc_2_radius*1000, Arc_2_start_angle, Arc_2_start_angle-AngleB_Range, 720, 2)
	Draw_half_circle(img, Arc_3_x*1000, Arc_3_y*1000, Arc_3_radius*1000, Arc_3_start_angle, Arc_3_start_angle-AngleC_Range, 720, 2)
	Draw_half_circle(img, Arc_4_x*1000, Arc_4_y*1000, Arc_4_radius*1000, Arc_4_start_angle, Arc_4_start_angle-AngleB_Range, 720, 2)
	#Draw_range_4    (img, -0, PI/2, 720, 2)
	Draw_range_4    (img, AngleB_Min, AngleB_Max, 720, 2)

#OpenCV显示机械臂逆解过程，及相关参数
def Opencv_Draw():
	img = np.zeros((img_height, img_width, 3), dtype=np.uint8) + 120
	origin_x=30
	origin_y=img_height-200
	origin_line_length=200
	cv2.line(img, (origin_x, origin_y), (origin_x,origin_y-180),       (0, 0, 0), 2)
	cv2.line(img, (origin_x, origin_y), (origin_line_length,origin_y), (0, 0, 0), 2)
	PointB_x=0.14*math.cos(Target_Angle_B_Inverse/57.3)*1000
	PointB_y=0.14*math.sin(Target_Angle_B_Inverse/57.3)*1000
	cv2.line(img, (origin_x, origin_y), (int(PointB_x)+origin_x,origin_y-int(PointB_y)), (255, 255, 0), 2)
	PointC_x=PointB_x+0.16*math.cos(Target_Angle_C_Inverse/57.3-PI+Target_Angle_B_Inverse/57.3)*1000
	PointC_y=PointB_y+0.16*math.sin(Target_Angle_C_Inverse/57.3-PI+Target_Angle_B_Inverse/57.3)*1000

	cv2.circle(img, (int(ARM_Target_X*1000)+origin_x, origin_y-int(ARM_Target_Z*1000)), 3, (0, 0, 255), 2)
	cv2.line(img, (int(PointB_x)+origin_x,origin_y-int(PointB_y)), (int(PointC_x)+origin_x,origin_y-int(PointC_y)), (255, 255, 0), 2)
	
	cv2.putText(img, 'TargetX: '+str(int(ARM_Target_X*1000)), (400, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'TargetY: '+str(int(ARM_Target_Z*1000)), (500, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'PointX: '+str(int(PointC_x)), (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'PointY: '+str(int(PointC_y)), (500, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'AngleB: '+str(int(Target_Angle_B_Inverse)), (400, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)#与X轴正轴的夹角
	cv2.putText(img, 'AngleC: '+str(int(Target_Angle_C_Inverse)), (500, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)#与X轴正轴的夹角
	cv2.putText(img, 'JointB: '+str(int(-90+Target_Angle_B_Inverse)), (400, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)#与初始位置的夹角
	cv2.putText(img, 'JointC: '+str(int(Target_Angle_C_Inverse-22)), (500, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)#与初始位置的夹角

	cv2.putText(img, 'R1: '+str(int(R1*1000)), (400, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'R2: '+str(int(R2*1000)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'R3: '+str(int(Arc_2_radius*1000)), (400, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'R4: '+str(int(Arc_4_radius*1000)), (500, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

	cv2.putText(img, 'Judge: ',                (400, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge1:
		cv2.putText(img, 'Judge1: Radius1<R2', (400, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge1: Radius1>R2', (400, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge2:
		cv2.putText(img, 'Judge2: Radius2<Arc_2_radius', (400, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge2: Radius2>Arc_2_radius', (400, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge3:
		cv2.putText(img, 'Judge3: Radius3>R2', (400, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge3: Radius3<R2', (400, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge4:
		cv2.putText(img, 'Judge4: Radius4>Arc_4_radius', (400, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge4: Radius4<Arc_4_radius', (400, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge5:
		cv2.putText(img, 'Judge5: PointY<-150', (400, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge5: PointY>-150', (400, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	if Judge6:
		cv2.putText(img, 'Judge6: AngleB+AngleC<90', (400, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
	else:
		cv2.putText(img, 'Judge6: AngleB+AngleC>90', (400, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

	cv2.putText(img, 'Radius1: '+str(int(Radius1*1000)), (400, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'Radius2: '+str(int(Radius2*1000)), (500, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'Radius3: '+str(int(Radius3*1000)), (400, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'Radius4: '+str(int(Radius4*1000)), (500, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

	cv2.putText(img, 'w, a, s, d to move point',    (400, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'w: up',    (400, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 's: down',  (500, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'a: left',  (400, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'd: right', (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	cv2.putText(img, 'q: quit',  (400, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

	Draw_Range_Judge(img)
	cv2.imshow("img", img)

step=0.001
#主函数
if __name__=="__main__":
	while 1:
		key=cv2.waitKey(1)


		if key == '\x03':
		            break
		elif key == ord('w') or key == ord('W'):
			ARM_Target_Z=ARM_Target_Z+step
		elif key == ord('a') or key == ord('A'):
			ARM_Target_X=ARM_Target_X-step
		elif key == ord('s') or key == ord('S'):
			ARM_Target_Z=ARM_Target_Z-step
		elif key == ord('d') or key == ord('D'):
			ARM_Target_X=ARM_Target_X+step
		elif key == ord('z') or key == ord('Z'):
			step=step+0.001
		elif key == ord('x') or key == ord('X'):
			step=step-0.001
		elif key == ord('q') or key == ord('Q'):
			break
		if step<0:
			step=0

		ARM_Inverse_Solution(ARM_Target_X, ARM_Target_Y, ARM_Target_Z)		
		Opencv_Draw()
