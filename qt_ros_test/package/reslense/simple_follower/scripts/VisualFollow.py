#!/usr/bin/env python 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作，也可以指定python3
#2:Python.X 源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

import rospy #引用ROS的Python接口功能包
import message_filters #https://blog.csdn.net/chishuideyu/article/details/77479758
import cv2, cv_bridge #引用opencv功能包。cv_bridge是ROS图像消息和OpenCV图像之间转换的功能包
import numpy as np #引用数组功能包
from sensor_msgs.msg import Image #引用ROS内的图片消息格式
from sensor_msgs.msg import CompressedImage #引用ROS内的图片消息格式
from geometry_msgs.msg import Twist #引用ROS的速度消息格式
from dynamic_reconfigure.server import Server
from simple_follower.cfg import HSV_ParamentConfig
from std_msgs.msg import Int8
#定义视觉跟踪类
class VisualFollower:
    def __init__(self):#类初始化
                #从参数服务器获取相关参数，这些参数在launch文件中定义
                self.vertAngle=rospy.get_param('~visual_angle/vertical') #定义摄像头垂直可视角度大小
                self.horizontalAngle=rospy.get_param('~visual_angle/horizontal') #定义摄像头水平可视角度大小
                self.targetDistance=rospy.get_param('~targetDistance') #定义目标距离
                self.velocity_foward_restrict=rospy.get_param('~velocity_restrict/forward') #前进后退速度限幅
                self.velocity_turn_restrict=rospy.get_param('~velocity_restrict/turn')      #转向速度限幅
                self.targetUpper = np.array([255, 255, 255])
                self.targetLower = np.array([118, 110, 16]) 
                self.X_correct = 10
                self.Distance_KP = rospy.get_param('~velocity_multiple/forward') #前进后退速度放大倍率
                self.Angle_KP = rospy.get_param('~velocity_multiple/turn')      #转向速度发大倍率
                self.Distance_KD = rospy.get_param('~velocity_multiple/forward_kd')
                self.Angle_KD = rospy.get_param('~velocity_multiple/turn_kd')
                self.Last_Distance_Bias = 0
                self.Last_angleX = 0
                self.twist = Twist() #创建速度控制命令变量
                self.bridge = cv_bridge.CvBridge() #OpenCV与ROS的消息转换类

                #message_filters的作用是把订阅的数据同步后再在message_filters的回调函数中使用
                im_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
                dep_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
                self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
                self.timeSynchronizer.registerCallback(self.VisualFollow)
        
                self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) #速度控制命令发布者
    '''
                self.srv = Server(HSV_ParamentConfig, self.config_callback) 

    def config_callback(self, config, level):
        print(config)
        self.targetUpper[0]=config.HSV_H_MAX
        self.targetLower[0]=config.HSV_H_MIN
        self.targetUpper[1]=config.HSV_S_MAX
        self.targetLower[1]=config.HSV_S_MIN
        self.targetUpper[2]=config.HSV_V_MAX
        self.targetLower[2]=config.HSV_V_MIN
        print("HSV_H_MAX:"+str(self.targetUpper[0]))
        print("HSV_H_MIN:"+str(self.targetLower[0]))
        print("HSV_S_MAX:"+str(self.targetUpper[1]))
        print("HSV_S_MIN:"+str(self.targetLower[1]))
        print("HSV_V_MAX:"+str(self.targetUpper[2]))
        print("HSV_V_MIN:"+str(self.targetLower[2]))
        return config
    '''

    def publish_flag(self):
		visual_follow_flag=Int8()
		visual_follow_flag.data=1
		rospy.sleep(1.)
		visualfwflagPublisher.publish(visual_follow_flag)
		rospy.loginfo('a=%d',visual_follow_flag.data)
		print("1111111111111111111111111111111111111111111111111111111111111")

    #视觉跟踪实现函数
    def VisualFollow(self, image_data, depth_data):
                #图像转换与预处理            
                frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')            #ROS图像转OpenCV图像
                depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#ROS图像转OpenCV图像
                frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)            #降低图像分辨率，以提高程序运行速度        
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                depthFrame = cv2.resize(depthFrame, (320,240), interpolation=cv2.INTER_AREA)  #降低图像分辨率，以提高程序运行速度        
                frame_height, frame_width, frame_channels = frame.shape #获得图像尺寸高度、宽度、通道数
                #cv2.imshow("frame", frame)
                #cv2.imshow("depthFrame", depthFrame)

                #图像二值化
                #self.targetUpper=np.array([138, 255, 157]) #红色
                #self.targetLower=np.array([120, 181, 68])  #红色
                frame_threshold = cv2.inRange(frame, self.targetLower, self.targetUpper)

                #膨胀腐蚀
                kernel_width=5;  
                kernel_height=5; 
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))
                frame_threshold = cv2.erode(frame_threshold, kernel)
                frame_threshold = cv2.dilate(frame_threshold, kernel)
                frame_threshold = cv2.dilate(frame_threshold, kernel)
                frame_threshold = cv2.erode(frame_threshold, kernel)
                #cv2.imshow("frame_threshold", frame_threshold) #显示预处理结果

                #寻找轮廓API，功能：输入图片，返回原图、轮廓对象、轮廓父子结构
                hierarchy = cv2.findContours(frame_threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.RETR_EXTERNAL：只检测外轮廓，cv2.CHAIN_APPROX_SIMPLE)：只保留轮廓的交点

                #如果找到了轮廓，则保留最大面积轮廓—>规划一个比轮廓形状小的区域—>获得该区域的深度信息(即与目标的距离)—>从深度信息、区域中心坐标判断并给twist赋值以进行前进后退左右转
                try:
                 contours_sorted=sorted(contours, key=cv2.contourArea, reverse=True) #按照轮廓面积进行排序
                 cv2.drawContours(frame, contours_sorted, 0, (0,0,255), 3)           #画出第一个轮廓，因为已排序，所以即为最大的轮廓
                 
                 (x, y),radius = cv2.minEnclosingCircle(contours_sorted[0]) #对最大轮廓求最小外接圆，并返回中心坐标(浮点型)，半径(浮点型)
                 rospy.loginfo("X:"+str(x))
                 x=x-12
                 x_float=x #保留浮点型，用于后面计算角度
                 y_float=y #保留浮点型，用于后面计算角度
                 x=int(x) #转为整型才能用于图像处理
                 y=int(y) #转为整型才能用于图像处理
                 radius=int(radius) #转为整型才能用于图像处理
                 #cv2.imshow("frame1", frame) #显示识别效果
                 cv2.circle(frame, (x, y), radius, (0, 255, 255), 3) #画出最小外接圆
                 target_radius=radius/3 #用于简单求轮廓内接矩形
                 depthObject = depthFrame[(y-target_radius):(y+target_radius), (x-target_radius):(x+target_radius)]#截取目标区域的深度图， 用于取区域内深度信息, frame[(height_start:height_end), (width_start:height_end)]
                 cv2.rectangle(frame,(x-target_radius,y-target_radius),(x+target_radius, y+target_radius), (0,255,0), 2)#画出轮廓内接矩形
                 depthArray = depthObject[~np.isnan(depthObject)]#获取区域内非空的深度信息
                 averageDistance = np.mean(depthArray)           #求平均得到与目标的距离，单位:毫米

                 #根据区域中心点坐标与摄像头可视角度计算目标与摄像头中心的偏角
                 angleX=self.horizontalAngle*(0.5-x_float/frame_width)
                 angleY=self.vertAngle*(0.5-y_float/frame_height)

                 #打印偏角、距离信息
                 rospy.loginfo("angleX:"+str(angleX))
                 rospy.loginfo("angleY:"+str(angleY))
                 rospy.loginfo("distance:"+str(averageDistance))

                 #深度摄像头在距离太近时是直接没有深度信息的，如果平均距离小于1则判断深度信息错误，不发布速度控制命令
                 if(averageDistance>1):
                  Distance_Bias=averageDistance-self.targetDistance #求实际距离与目标距离的偏差

                  #偏差较小时不进行移动
                  if(angleX<10 and angleX>-10):
                   angleX=0
                  if (Distance_Bias<200 and Distance_Bias>-200):
                   Distance_Bias=0

                  Distance_Bias_D = Distance_Bias - self.Last_Distance_Bias
                  angleX_D = angleX - self.Last_angleX

                  #when target is little, amplify velocity by amplify error.
                  if (Distance_Bias>0 and self.targetDistance<1200):
                    Distance_Bias=Distance_Bias*(1200/self.targetDistance)*0.7

                  #根据偏差大小给速度控制命令赋值
                  #前进为正，后退为负，单位：米
                  self.twist.linear.x  = Distance_Bias/1000*self.Distance_KP - Distance_Bias_D/1000*self.Distance_KD
                  #左转为正，右转为负，单位：弧度
                  self.twist.angular.z = angleX/180*3.14*self.Angle_KP - angleX_D/180*3.14*self.Angle_KD

                  self.Last_Distance_Bias = Distance_Bias
                  self.Last_angleX = angleX

                  #前进后退速度限幅
                  if   (self.twist.linear.x> self.velocity_foward_restrict):
                    self.twist.linear.x= self.velocity_foward_restrict
                  elif (self.twist.linear.x<-self.velocity_foward_restrict):
                    self.twist.linear.x=-self.velocity_foward_restrict

                  #转向速度限幅
                  if   (self.twist.linear.z> self.velocity_turn_restrict):
                    self.twist.linear.x= self.velocity_foward_restrict
                  elif (self.twist.linear.z<-self.velocity_turn_restrict):
                    self.twist.linear.x=-self.velocity_turn_restrict
                
                #没有找到轮廓，速度控制命令置零
                except IndexError:
                 rospy.logwarn("contours not found")
                 self.twist.linear.x  = 0 
                 self.twist.angular.z = 0

                #发布控制命令
                self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("frame2", frame) #显示识别效果
                cv2.waitKey(1)
                      
if __name__ == '__main__': #这段判断的作用是，如果本py文件是直接运行的则判断通过执行if内的内容，如果是import到其他的py文件中被调用(模块重用)则判断不通过
  visualfwflagPublisher = rospy.Publisher('/visual_follow_flag', Int8, queue_size =1)
  rospy.init_node("opencv") #创建节点
  rospy.loginfo("OpenCV VisualFollow node started") #打印ROS消息说明节点已开始运行
  Visual_follower=VisualFollower() 
  print("123123123")
  rospy.spin() #相当于while(1),当订阅者接收到新消息时调用回调函数

