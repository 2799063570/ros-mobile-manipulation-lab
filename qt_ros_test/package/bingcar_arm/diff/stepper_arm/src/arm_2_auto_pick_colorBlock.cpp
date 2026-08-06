#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include </usr/include/opencv2/highgui/highgui_c.h> //树莓派(Raspberry)、工控机(IPC)、虚拟机
//#include </usr/include/opencv4/opencv2/highgui/highgui_c.h> //jetson Nano/NX/TX2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <stepper_arm/arm_color_block_paramsConfig.h>

#include <pthread.h>
#include <arm_2_control.h>

using namespace std;
using namespace cv;
 
//ROS图片话题
string rgb_image_topic;
//opencv图片对象
Mat image, HSV_image, final_image;
//ROS里程计话题
string odom_topic;

/****可以使用rqt动态调参的变量****/
//抓取色块标志位
int Pick_start=false;
double car_search_distance_max=3;
//机械臂初始位置
double grab_start_jointA = 1.570796;
double grab_start_position_x = 0.062;
double grab_start_position_y =-0.008;
//机械臂结束位置
double grab_end_jointA = -1.570796;
double grab_end_position_x = 0.062;
double grab_end_position_y = 0.062;

//机械爪初始张开程度
double hand_open_size=-0.7; 
//机械爪抓取色块夹紧程度
double hand_close_size=0.5; 
//色块识别阈值
int color=0;//0:dynamic, 1:green, 2:blue, 3:yellow. 4:red
int dynamic_hsv[6]={0,  0,   0,   0,   0,   0  }; //[h_min, s_min, v_min, h_max, s_max, v_max]
int green_hsv  [6]={60, 35,  0,   80,  180, 255}; //[h_min, s_min, v_min, h_max, s_max, v_max]
int blue_hsv   [6]={80, 90,  100, 130, 200, 255}; //[h_min, s_min, v_min, h_max, s_max, v_max]
int yellow_hsv [6]={15, 110, 30,  50,  255, 255}; //[h_min, s_min, v_min, h_max, s_max, v_max]
int red_hsv    [6]={0,  100, 53,  18,  223, 255}; //[h_min, s_min, v_min, h_max, s_max, v_max]
//膨胀腐蚀大小
int dilate_erode_size=7;
//期望识别到的目标的面积的大小，代表距离。因为RGB相机没有深度信息，只能在已知目标大小的情况下，通过判断面积大小计算近似距离(近大远小)
double target_areaSize=22000; 
//期望色块位于画面中心向上偏移多少像素点
double target_y_bias=0; 
//寻找色块时小车的转速
double car_search_turn=0.1;
double car_search_foward=0.1;
//抓取色块时机械臂的运动限幅参数，前进后退、上升下降、左转右转
float Move_step_max=0.4, Turn_step_max=0.2;
float Move_step_min=0.0005, Turn_step_min=0.0005;
//抓取色块时机械臂运动靠近色块的PID参数
double turn_KP=0.05,   turn_KD=0;
double down_KP=0.01,   down_KD=0;
double foward_KP=0.01, foward_KD=0;
//寻找色块时小车的运动参数,
double Car_foward_KP;
/****可以使用rqt动态调参的变量****/

//机械臂小车寻找抓取色块的相关变量
double area;
double distance_bias, x_bias, y_bias;
double last_distance_bias, last_x_bias, last_y_bias;
double distance_step=0, x_step=0, y_step=0;
double foward_step, down_step;
double Car_foward_velocity, Car_turn_velocity;
bool foward_forb=false;
//机械臂小车寻找抓取色块过程中的各个状态变量
bool Car_stop=false, car_turn_locked=false, Arm_stop=true;
bool hand_on_color=false, arm_turn_locked=false, arm_locked=false, hand_closed=false;
int Car_turn_times=0;
//小车里程计，用于寻找色块功能
double odom_foward, odom_lateral, odom_turn;
//机械臂时间状态变量
ros::Time arm_position_move_time, arm_locked_time, hand_closed_time;

//rqt动态调参函数
void dynamic_reconfigure_callback(stepper_arm::arm_color_block_paramsConfig &config, uint32_t level);
//获取摄像头图像后的处理函数，主要功能实现在这里
void rgb_image_Callback(const sensor_msgs::ImageConstPtr& msg);
//获得小车里程计
void odom_Callback(const nav_msgs::Odometry& msg);
//opencv进度条调阈值回调函数，放弃使用，改由rqt调动态阈值
void on_trackbar(int, void*) {}

/**************************************************************************
Function: The main function
Input   : none
Output  : 无
函数功能： 主函数
入口参数： 无
返回  值： 无
**************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc,argv,"arm_2_auto_pick_colorBlock"); //初始化节点
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    ros::NodeHandle NodeHandle; //创建节点句柄
   
    //关节B默认角度(相对X轴，逆时针为正)
    NodeHandle.param<double>("Angle_B_bias",    Angle_B_bias,    1.571);
    //关节C默认角度(相对关节B，逆时针为正)
    NodeHandle.param<double>("Angle_C_bias",    Angle_C_bias,    0.384);
    //机械臂底座到地面的距离
    NodeHandle.param<double>("arm_base_height", arm_base_height, 0);
    //图片话题名
    NodeHandle.param<string>("rgb_image_topic", rgb_image_topic, "/usb_cam/image_raw");
    //里程计话题名
    NodeHandle.param<string>("odom_topic",      odom_topic,      "/odom");

    //摄像头图片话题订阅者，用于识别抓取色块
    ros::Subscriber rgb_image_Sub; 
    rgb_image_Sub   = NodeHandle.subscribe(rgb_image_topic, 20, &rgb_image_Callback);
    //机械臂关节姿态信息发布者，用于控制机械臂
    ros::Publisher Arm_2_JointStates_Pub;
    Arm_2_JointStates_Pub = NodeHandle.advertise<sensor_msgs::JointState>("Arm_2_JointStates", 10);
    //速度命令话题发布者，用于控制小车运动寻找色块
    ros::Publisher cmd_vel_Pub;
    cmd_vel_Pub = NodeHandle.advertise<geometry_msgs::Twist>("cmd_vel_ori", 10);
    //里程计话题订阅者，用于识别抓取色块
    ros::Subscriber odom_Sub; 
    odom_Sub   = NodeHandle.subscribe(odom_topic, 20, &odom_Callback);

    //动态调参初始化，用于控制抓取色块、改变目标色块颜色
    dynamic_reconfigure::Server<stepper_arm::arm_color_block_paramsConfig> server;
    dynamic_reconfigure::Server<stepper_arm::arm_color_block_paramsConfig>::CallbackType f;
    f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
    server.setCallback(f);

    Joint_A=grab_start_jointA;
    ArmC_End_Position_X=grab_start_position_x;
    ArmC_End_Position_Y=grab_start_position_y;
    Joint_Hand_left1  =  hand_open_size, Joint_Hand_left2  =  hand_open_size, 
    Joint_Hand_right1 = -hand_open_size, Joint_Hand_right2 = -hand_open_size;
    arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
  
    ros::Rate loopRate(70);//频率70Hz
    while(ros::ok())
    {
        //发布机械臂关节姿态信息，用于控制机械臂
        sensor_msgs::JointState Arm_2_JointStates;
        if(Joint_A>AngleA_Max)Joint_A=AngleA_Max;
        if(Joint_A<AngleA_Min)Joint_A=AngleA_Min;
        Arm_2_JointStates.position.push_back(Joint_A);
        //减去Angle_B_bias是因为moveit的关节初始位置为0
        Arm_2_JointStates.position.push_back(Joint_B-Angle_B_bias);
        //减去Angle_C_bias是因为moveit的关节初始位置为0
        Arm_2_JointStates.position.push_back(Joint_C-Angle_C_bias);
        Arm_2_JointStates.position.push_back(Joint_End);
        Arm_2_JointStates.position.push_back(Joint_Hand_left1);
        Arm_2_JointStates.position.push_back(Joint_Hand_left2);
        Arm_2_JointStates.position.push_back(Joint_Hand_right1);
        Arm_2_JointStates.position.push_back(Joint_Hand_right2);
        Arm_2_JointStates.position.push_back(Joint_Grasper);
        Arm_2_JointStates_Pub.publish(Arm_2_JointStates);   

        //发布速度命令话题，用于控制小车运动寻找色块
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x =Car_foward_velocity;
        cmd_vel.linear.y =0;
        cmd_vel.linear.z =0;
        cmd_vel.angular.x=0;
        cmd_vel.angular.y=0;
        cmd_vel.angular.z=Car_turn_velocity;
        cmd_vel_Pub.publish(cmd_vel); 

        ros::spinOnce();
        loopRate.sleep();
    } 
    return 0;
}

/**************************************************************************
Function: rqt dynamically calls the parameter callback function
Input   : none
Output  : none
函数功能： rqt动态调参回调函数
入口参数： 动态调参对象，
返回  值： 无
**************************************************************************/
void dynamic_reconfigure_callback(stepper_arm::arm_color_block_paramsConfig &config, uint32_t level)
{
    Pick_start=config.Pick_start;

    car_search_distance_max=config.car_search_distance_max;
    grab_start_jointA=config.grab_start_jointA;
    grab_start_position_x=config.grab_start_position_x;
    grab_start_position_y=config.grab_start_position_y;
    grab_end_jointA=config.grab_end_jointA;
    grab_end_position_x=config.grab_end_position_x;
    grab_end_position_y=config.grab_end_position_y;

    hand_open_size=config.hand_open_size;
    hand_close_size=config.hand_close_size;

    color=config.color;

    //使用ROS的rqt工具进行动态调阈值
    dynamic_hsv[0]=config.HSV_H_MIN;
    dynamic_hsv[1]=config.HSV_S_MIN;
    dynamic_hsv[2]=config.HSV_V_MIN;
    dynamic_hsv[3]=config.HSV_H_MAX;
    dynamic_hsv[4]=config.HSV_S_MAX;
    dynamic_hsv[5]=config.HSV_V_MAX;
    dilate_erode_size=config.dilate_erode_size;
    /*cout<<"color= "<<color<<endl;
    cout<<"levle:"<<level<<endl;*/
    
    car_search_foward=config.car_search_foward;

    target_areaSize=config.target_areaSize;

    target_y_bias=config.target_y_bias;

    Move_step_max=config.Move_step_max;
    Turn_step_max=config.Turn_step_max;
    Move_step_min=config.Move_step_min;
    Turn_step_min=config.Turn_step_min;

    turn_KP=config.turn_KP;
    turn_KD=config.turn_KD;
    down_KP=config.down_KP;
    down_KD=config.down_KD;
    foward_KP=config.foward_KP;
    foward_KD=config.foward_KD;

    Car_foward_KP=config.Car_foward_KP;
}

/**************************************************************************
Function: RGB image topic subscription callback function
Input   : none
Output  : none
函数功能： RGB图像话题订阅回调函数
入口参数： 无
返回  值： 无
**************************************************************************/
void rgb_image_Callback(const sensor_msgs::ImageConstPtr& msg)
{  
    /*零、ROS图片话题转换为OpenCV图片格式*************************/
    try 
    {
        cv_bridge::CvImagePtr cv_ptr;  //ROS图像格式
        //将ROS话题数据转换为ROS图像格式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //将ROS图像格式转换为opencv图像格式
        cv_ptr->image.copyTo(image);

        //压缩图片，提高图片处理速度，同时减轻远程运行程序时的网络带宽压力
        CvSize size;
        size.width = image.cols/2;
        size.height = image.rows/2;
        resize(image,image,size,0,0, CV_INTER_AREA);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    /*零、ROS图片话题转换为OpenCV图片格式*************************/


    /*一、图像处理：识别色块部分**********************************/
    cvtColor(image, HSV_image, CV_BGR2HSV);//RGB图像转换为HSV图像
    
    /*0.在原图上标记出目标位置*/
    double target_x=image.cols/2, target_y=target_y_bias; //控制机械臂使色块位于画面内的目标位置
    circle(image, Point(target_x, target_y), 10, Scalar(0, 0, 255), 1); //画色块轮廓外接圆    
    line(image, Point(target_x-10,target_y),Point(target_x+10,target_y), Scalar(0, 0, 255),1,8,0);//在目标位置画十字
    line(image, Point(target_x,target_y-10),Point(target_x,target_y+10), Scalar(0, 0, 255),1,8,0);//在目标位置画十字
    
    /*1.根据颜色阈值进行图片二值化*/
    if(color==0) //根据HSV颜色空间下的自定义颜色阈值(即动态调阈值)，进行图像二值化，仅保留指定自定义颜色
    {
        inRange(HSV_image, Scalar(dynamic_hsv[0], dynamic_hsv[1], dynamic_hsv[2]), 
                             Scalar(dynamic_hsv[3], dynamic_hsv[4], dynamic_hsv[5]), final_image); 
    }
    else
    {
        if(color==1) //根据HSV颜色空间下的预置绿色阈值，进行图像二值化，仅保留绿色
        {
            inRange(HSV_image, Scalar(green_hsv[0], green_hsv[1], green_hsv[2]), 
                                 Scalar(green_hsv[3], green_hsv[4], green_hsv[5]), final_image); 
        }
        if(color==2) //根据HSV颜色空间下的预置蓝色阈值，进行图像二值化，仅保留蓝色
        {
            inRange(HSV_image, Scalar(blue_hsv[0], blue_hsv[1], blue_hsv[2]), 
                                 Scalar(blue_hsv[3], blue_hsv[4], blue_hsv[5]), final_image); 
        }
        if(color==3) //根据HSV颜色空间下的预置黄色阈值，进行图像二值化，仅保留黄色
        {
            inRange(HSV_image, Scalar(yellow_hsv[0], yellow_hsv[1], yellow_hsv[2]), 
                                 Scalar(yellow_hsv[3], yellow_hsv[4], yellow_hsv[5]), final_image); 
        }      
        if(color==4) //根据HSV颜色空间下的预置红色阈值，进行图像二值化，仅保留红色
        {
            inRange(HSV_image, Scalar(red_hsv[0], red_hsv[1], red_hsv[2]), 
                                 Scalar(red_hsv[3], red_hsv[4], red_hsv[5]), final_image); 
        }
    }
     
    /*2.膨胀腐蚀清除图像杂质*/
    Mat element = getStructuringElement(MORPH_RECT, Size(dilate_erode_size, dilate_erode_size)); 
    dilate(final_image, final_image, element);
    erode(final_image, final_image, element);
    erode(final_image, final_image, element);
    dilate(final_image, final_image, element);

    //3.获得当前图像存在的所有轮廓(经过前面的处理，只有指定颜色的目标(色块)才有轮廓)
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy; 
    try
    {
      findContours(final_image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
    }
    catch (cv::Exception& e)
    {
      const char* err_msg = e.what();
      cout << "exception caught: " << err_msg << endl;
    }
    /*一、图像处理：识别色块部分**********************************/

    /*********如果视野内存在色块，则停止色块寻找，开始执行色块抓取*/
    /*********如果视野内存在色块，则停止色块寻找，开始执行色块抓取*/
    /*********如果视野内存在色块，则停止色块寻找，开始执行色块抓取*/
    /*二、图像处理：提取色块位置信息与可视化部分*******************/
    static int object_found=0;
    if(contours.size()>0) //如果轮廓对象不为空，则开始进行处理
    {
        if(object_found==0)cout<<GREEN<<"Object found."<<RESET<<endl<<endl; //只在第一次发现目标色块时打印
        object_found=1;

        int largest_contours_order=0; //所有轮廓中面积最大轮廓的序号
        
        /*1.提取色块位置信息*/
        //获得面积最大轮廓的序号
        double largest_area=0;
        for(int i=0;i<contours.size();i++)  
        {  
            if(cv::contourArea(contours[i])>largest_area)
            {
                largest_area=cv::contourArea(contours[i]);
                largest_contours_order=i;
            }      
        }
        //获得面积最大轮廓的面积大小
        area= cv::contourArea(contours[largest_contours_order]);
        //cout<<"area: "<<area<<endl; 
        //面积最大轮廓的外接圆半径
        float radius=0; 
        //面积最大轮廓的中心位置(横轴为x，纵轴为y，右上角为原点)
        Point2f center;
        //求面积最大轮廓外接圆，获得轮廓中心位置
        minEnclosingCircle(contours[largest_contours_order], center, radius);  

        //求获得当前色块与目标位置、距离的偏差
        x_bias=image.cols/2-center.x; //横轴偏差，色块中心在图像中心的左边为正，单位为分辨率
        y_bias=image.rows/2-center.y-target_y; //纵轴偏差，色块中心在目标位置的上边为正，单位为分辨率
        distance_bias=target_areaSize-area; //目标面积-色块面积=距离偏差，色块过远为正，单位为分辨率*分辨率
        /*cout<<"distance_bias: "<<distance_bias<<endl; 
        cout<<"x_bias: "<<x_bias<<endl; 
        cout<<"y_bias: "<<y_bias<<endl;*/ 

        /*2.色块位置信息可视化*/
        /*2.1.1在原图上标记出识别到的指定颜色目标*/
        circle(image, Point(center.x, center.y), radius, Scalar(0, 0, 255), 5); //画色块轮廓外接圆    
        line(image, Point(center.x-10,center.y),Point(center.x+10,center.y), Scalar(0, 0, 255),1,8,0);//在色块中心画十字
        line(image, Point(center.x,center.y-10),Point(center.x,center.y+10), Scalar(0, 0, 255),1,8,0);//在色块中心画十字
        
        /*2.2.在原图上显示当前色块与目标位置(摄像头中心)、距离的偏差*/
        //定义用于显示的文字变量
        /*string area_txt="area: ";
        string bias_area_txt="bias_area: ";
        string bias_x_txt="bias_x: ";
        string bias_y_txt="bias_y: ";
        //初始化用于显示文字的字体格式
        CvFont font;
        double hScale=0.5, vScale=0.5;
        int    lineWidth=1.5;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,hScale,vScale,0,lineWidth,8);
        //限制要显示的文字在图像上的位置在图像大小内
        int txt_position_x, txt_position_y;
        txt_position_x=10;
        txt_position_y=50;
        if(txt_position_x<5)txt_position_x=5;
        if(txt_position_x>image.cols-50)txt_position_x=image.cols-50;
        if(txt_position_y<50)txt_position_y=50;
        if(txt_position_y>image.rows-60)txt_position_y=image.rows-60;
        //显示文字需要先转换图像格式
        IplImage tmp = IplImage(image);
        CvArr* image_=(CvArr*)&tmp;
        //显示当前色块面积
        area_txt = area_txt + to_string((int)area);
        cvPutText(image_, area_txt.data(), cvPoint(txt_position_x, txt_position_y-30), &font, cvScalar(0,0,0,1));
        //显示目标色块面积与当前色块面积的偏差
        bias_area_txt = bias_area_txt + to_string((int)distance_bias);
        cvPutText(image_, bias_area_txt.data(), cvPoint(txt_position_x, txt_position_y-10), &font, cvScalar(0,0,0,1));
        //显示图像中心与当前色块中心的横轴偏差
        bias_x_txt = bias_x_txt + to_string((int)x_bias);
        cvPutText(image_, bias_x_txt.data(), cvPoint(txt_position_x, txt_position_y+10), &font, cvScalar(0,0,0,1));
        //显示图像中心与当前色块中心的纵轴偏差
        bias_y_txt = bias_y_txt + to_string((int)y_bias);
        cvPutText(image_, bias_y_txt.data(), cvPoint(txt_position_x, txt_position_y+30), &font, cvScalar(0,0,0,1));
        //转换图像格式
        image = cv::cvarrToMat(image_);*/
    /*二、图像处理：提取色块位置信息与可视化部分*******************/

    /*三、如果发现了色块，首先控制小车移动，使小车移动到与色块合适的距离，然后再计算机械臂的运动数值，用于抓取色块；
         如果当前视野内没有发现色块，则控制小车运动寻找色块*/
        /*1.首先控制小车移动，使小车移动到与色块合适的距离*/
        if(Car_stop==false && Pick_start==true)
        {
            /*1.1.如果横轴偏差比较大，控制小车旋转，使偏差减小到0*/
            if(x_bias>70 || x_bias<-70)
            {
                Car_foward_velocity=x_bias/400*Car_foward_KP;
            }
            else
            {
                Car_foward_velocity=0;
                Car_stop=true, Arm_stop=false;
                cout<<GREEN<<"Car location complete."<<RESET<<endl<<endl;
            }
        }
        
        /*2.计算机械臂的运动数值，用于抓取色块*/
        if(Car_stop==true && Arm_stop==false && arm_locked==false)
        {
            /*2.1.如果距离偏差(色块与机械臂的距离)大于1000，则对y轴偏差做处理，求机械臂上下方向运动值*/
            if((distance_bias>1000)&& arm_locked==false)
            {
                //如果偏差较大，说明机械爪还没有靠近目标，继续运动进行调整
                y_step = (y_bias*down_KP + (y_bias-last_y_bias)*down_KD)*0.0005;  
            }
            /*2.2.如果距离偏差(色块与机械臂的距离)小于1000，则锁定机械臂的前后上下方向的位移。下一步机械爪将夹紧*/
            else if(arm_locked==false)
            {
                y_step = 0;
                arm_locked=true;
                arm_locked_time=ros::Time::now();
                cout<<GREEN<<"Arm_hand can close now."<<RESET<<endl<<endl;
            }
            //if(area<target_areaSize*0.8 && y_step<0.00 && ArmC_End_Position_X<0.145 &&arm_in_min_height==1) y_step=distance_step*1; //机械爪离目标远时，禁止机械臂后退
            /*2.3.如果横轴偏差大小大于5，则对偏差做处理，求机械臂左右旋转的运动值*/
            if((x_bias>5 || x_bias<-5  )&& arm_turn_locked==false)
            {
               x_step = (x_bias*turn_KP + (x_bias-last_distance_bias)*turn_KD)*0.01;      
            }
            else 
            {
                x_step = 0;
            }
            //当机械臂靠近色块后，转向偏差的认为已经稳定，后续禁止转向运动
            if(distance_bias<5000 && distance_bias>-5000) 
                arm_turn_locked=true;
            /*2.4.限制机械臂前后左右上下方向运动值的最大值和最小值*/
            //前后上下移动的最大值(单位：m)，转向移动的最大值(单位：rad)          
            if(x_step> Turn_step_max)       x_step= Turn_step_max;
            if(x_step<-Turn_step_max)       x_step=-Turn_step_max;
            if(y_step> Move_step_max)       y_step= Move_step_max;
            if(y_step<-Move_step_max)       y_step=-Move_step_max;
            if(distance_step> Move_step_max)distance_step= Move_step_max;
            if(distance_step<-Turn_step_max)distance_step=-Turn_step_max;
            //前后上下移动的最小值(单位：m)，转向移动的最小值(单位：rad)
            if(x_step< Turn_step_min && x_step> 0)x_step= Turn_step_min;
            if(x_step>-Turn_step_min && x_step< 0)x_step=-Turn_step_min;             
            if(y_step< Move_step_min && y_step> 0)y_step= Move_step_min;
            if(y_step>-Move_step_min && y_step< 0)y_step=-Move_step_min;
            if(distance_step< Move_step_min && distance_step> 0)distance_step= Move_step_min;   
            //机械臂不允许向上移动，提高机械臂向色块靠近的稳定性
            if(distance_step<0)distance_step=0;

            /*cout<<"distance_step: "<<distance_step<<endl; 
            cout<<"x_step: "<<x_step<<endl; 
            cout<<"y_step: "<<y_step<<endl; */
            
            //用于KD控制
            last_distance_bias=distance_bias;
            last_x_bias=x_bias;
            last_y_bias=y_bias;
        }      
    }
    
    /*******如果轮廓对象为空，则说明当前视野内没有目标颜色，执行寻找色块功能*******/
    else //对应判断if(contours.size()>0) 
    {
        /*1.机械臂前后左右上下方向运动值置0，即停止色块抓取*/
        distance_step=0;x_step=0;y_step=0;
        Joint_Hand_left1  =  hand_open_size, Joint_Hand_left2  =  hand_open_size, 
        Joint_Hand_right1 = -hand_open_size, Joint_Hand_right2 = -hand_open_size;
        object_found=0;
        cout<<RED<<"no object found."<<RESET<<endl;   

        /*2.寻找色块*/  
        if(Pick_start==true && Arm_stop==true && foward_forb==false)//判断是否下达了抓取色块命令
        {          
            Car_foward_velocity=car_search_foward;
        }
        else 
        {
            Car_foward_velocity=0;
            Car_turn_velocity=0; 
        }    
    }
    /*三、如果发现了色块，首先控制小车移动，使小车移动到与色块合适的距离，然后再计算机械臂的运动数值，用于抓取色块；
         如果当前视野内没有发现色块，则控制小车运动寻找色块*/

    /*四、根据机械臂前后左右上下方向运动值决定机械臂应该执行什么动作*/
    /*1.1.机械臂前后上下方向运动值不为0，则控制机械臂靠近色块*/
    if(Arm_stop==false && arm_locked==false)
    {
        //cout<<"distance_step!=0 && y_step!=0"<<endl;
        int error_foward, error_down;     
        error_foward=Position_move(foward, 0.01*foward_KP);
        /*cout<<"error_foward: "<<error_foward<<endl;
        cout<<"error_foward&(1<<5): "<<(error_foward&(1<<5))<<endl;
        cout<<"error_foward&(1<<3): "<<(error_foward&(1<<3))<<endl<<endl;*/

        error_down=Position_move(down, -y_step);
        /*cout<<"error_down: "<<error_down<<endl;
        cout<<"error_down&(1<<5): "<<(error_down&(1<<5))<<endl; 
        cout<<"error_down&(1<<3): "<<(error_down&(1<<3))<<endl<<endl; */
        
    }
    /*1.2.机械臂左右方向运动值不为0，则控制机械臂旋转*/
    if(x_step!=0)
    {
        //cout<<"x_step!=0"<<endl;
        Joint_A = Joint_A + x_step;
    }
  
    /*2.1.如果机械爪锁定标志位置1，代表已经抓取到色块，控制机械臂返回初始位置*/
    if(hand_closed==true)
    {
        //机械爪之前色块后 
        if(( ros::Time::now() - hand_closed_time).toSec()<2.0)
        {
            //0-2秒：机械臂回到默认位置
            ArmC_End_Position_X=grab_start_position_x;ArmC_End_Position_Y=grab_start_position_y;
            arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<5.0)
        {
            //2-5秒：机械臂底座旋转到终点
            Joint_A=grab_end_jointA;
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<6.0)
        {
            //5-6秒：机械臂前进到目标点X坐标
            ArmC_End_Position_X=grab_end_position_x;
            ArmC_End_Position_Y=grab_start_position_y;
            arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<7.0)
        {
            //6-7秒：机械臂下降到目标点Y坐标(地面)
            ArmC_End_Position_X=grab_end_position_x;
            ArmC_End_Position_Y=grab_end_position_y;
            arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<8.0)
        {
            //7-8秒：机械爪张开，放置色块
            Joint_Hand_left1  =  hand_open_size, Joint_Hand_left2  =  hand_open_size, 
            Joint_Hand_right1 = -hand_open_size, Joint_Hand_right2 = -hand_open_size;
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<9.0)
        {
            //8-9秒：机械臂上升回去
            ArmC_End_Position_X=grab_end_position_x;
            ArmC_End_Position_Y=grab_start_position_y;
            arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()<12.0)
        {
            //9-12秒：机械臂直接回归起点
            Joint_A=grab_start_jointA;
            ArmC_End_Position_X=grab_start_position_x;
            ArmC_End_Position_Y=grab_start_position_y;
            Joint_Hand_left1  =  hand_open_size, Joint_Hand_left2  =  hand_open_size, 
            Joint_Hand_right1 = -hand_open_size, Joint_Hand_right2 = -hand_open_size;
            arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(( ros::Time::now() - hand_closed_time).toSec()>12.0)
        {
            //回归后，清空相关标志位，为下一次抓取做准备
            Car_stop=false, Arm_stop=true, Car_turn_times=0;
            car_turn_locked=false, arm_turn_locked=false, arm_locked=false, hand_closed=false;
            Car_foward_velocity=0, Car_turn_velocity=0;
            foward_forb=false;
        }
    }
    /*2.2.如果机械臂锁定标志位置1，代表已经靠近色块，控制机械臂夹紧抓取色块*/
    if(arm_locked==true && hand_closed==false)
    {
        /*cout<<"_start_time: "<<arm_locked_time.toSec()<<endl;
        cout<<"ros::Time::now(): "<<ros::Time::now().toSec()<<endl;*/
        Joint_Hand_left1  =  hand_close_size;
        Joint_Hand_left2  =  hand_close_size;
        Joint_Hand_right1 = -hand_close_size;
        Joint_Hand_right2 = -hand_close_size;
        //cout<<"( ros::Time::now() - _start_time).toSec(): "<<( ros::Time::now() - arm_locked_time).toSec()<<endl;

        //夹紧后等待1秒钟，再置1机械爪锁定标志位
        if(( ros::Time::now() - arm_locked_time).toSec()>1 && hand_closed==false)//delay 1s
        {
            hand_closed=true; 
            hand_closed_time=ros::Time::now();
        }
    }

    /*五、如果抓取色块标志位置0，机械臂状态全部复位*/
    if(Pick_start==false)
    {
        Car_stop=false, Arm_stop=true, Car_turn_times=0;
        car_turn_locked=false, arm_turn_locked=false, arm_locked=false, hand_closed=false;
        Car_foward_velocity=0, Car_turn_velocity=0;
        foward_forb=false;
        Joint_A=grab_start_jointA;
        ArmC_End_Position_X=grab_start_position_x;
        ArmC_End_Position_Y=grab_start_position_y;
        Joint_Hand_left1  =  hand_open_size, Joint_Hand_left2  =  hand_open_size, 
        Joint_Hand_right1 = -hand_open_size, Joint_Hand_right2 = -hand_open_size;
        arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
    }

    /*cout<<endl<<GREEN<<"arm_turn_locked: "<<arm_turn_locked<<endl;
    cout<<GREEN<<"arm_locked: "<<arm_locked<<endl;
    cout<<GREEN<<"hand_closed: "<<hand_closed<<endl;
    cout<<GREEN<<"Car_stop: "<<Car_stop<<endl;
    cout<<GREEN<<"Arm_stop: "<<Arm_stop<<RESET<<endl<<endl;*/

    //显示原始图片，添加色块位置标注
    CvSize size;
    size.width = image.cols/2; //再次压缩图片，减轻远程运行程序时的网络带宽压力
    size.height = image.rows/2;
    resize(image,image,size,0,0, CV_INTER_AREA);
    imshow("rgb_image", image);
    //显示二值化图片，jetsonNano/NX目前无法显示单通道图像，暂时关闭
    //imshow("final_rgb_image", final_image);
    waitKey(1);
}

/**************************************************************************
Function: Odometer topic subscribe callback function
Input   : none
Output  : none
函数功能： 里程计话题订阅回调函数
入口参数： 无
返回  值： 无
**************************************************************************/
void odom_Callback(const nav_msgs::Odometry& msg)
{
    static double last_odom_turn, last_odom_foward;
    odom_foward  = msg.pose.pose.position.x-last_odom_foward; //Position //位置
    odom_lateral = msg.pose.pose.position.y;
    odom_turn    = msg.pose.pose.position.z-last_odom_turn;

    if(Pick_start==false)
    {
        last_odom_turn = msg.pose.pose.position.z;
        last_odom_foward = msg.pose.pose.position.x;
    }   
    if(hand_closed==true)
        last_odom_foward = msg.pose.pose.position.x;

    if(car_search_distance_max>0 && odom_foward>car_search_distance_max)//前进3m还没有发现色块，则停止运动
        foward_forb=true;
    if(car_search_distance_max<0 && odom_foward<car_search_distance_max)//前进3m还没有发现色块，则停止运动
        foward_forb=true;
    
    /*cout<<RED<<"odom_Callback"<<endl;
    cout<<RED<<"odom_foward:  "<<odom_foward<<endl;
    cout<<RED<<"odom_lateral: "<<odom_lateral<<endl;
    cout<<RED<<"odom_turn:    "<<odom_turn<<RESET<<endl;*/
}
