/**************************************************************************
功能：避障
**************************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <wheeltec_multi/avoid.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


using namespace std;
 
geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_avoid;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_data;    //速度控制信息数据

float distance1=100;    //障碍物距离
float dis_angleX=0;    //障碍物方向,前面为0度角，右边为正，左边为负	
double safe_distence;
double danger_distence;
double danger_angular;
double avoidance_kv;
double avoidance_kw;
double max_vel_x;
double max_vel_y;
double max_linear_vel;
double min_vel_x;
double max_vel_theta;
double min_vel_theta;
bool holonomic;
double control_rate;
double obstacle_timeout;
ros::Time last_obstacle_time;
bool obstacle_received=false;

/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const wheeltec_multi::avoid& msg)
{
	last_obstacle_time = ros::Time::now();
	obstacle_received = true;
	distance1 = msg.distance;
	dis_angleX = msg.angleX;
	if(dis_angleX>0)
		dis_angleX=3.1415-dis_angleX;
	else
		dis_angleX=-(dis_angleX+3.1415);
}

/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void cmd_vel_ori_Callback(const geometry_msgs::Twist& msg)
{
	cmd_vel_msg.linear.x = msg.linear.x;
	cmd_vel_msg.linear.y = msg.linear.y;
	cmd_vel_msg.angular.z = msg.angular.z;

	cmd_vel_data.linear.x = msg.linear.x;
	cmd_vel_data.linear.y = msg.linear.y;
	cmd_vel_data.angular.z = msg.angular.z;
}


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
	int temp_count = 0;    //计数变量
	ros::init(argc, argv, "avoidance");    //初始化ROS节点

	ros::NodeHandle node;    //创建句柄
	ros::NodeHandle private_nh("~");

	/***创建底盘速度控制话题发布者***/
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher avoidance_state_pub = node.advertise<std_msgs::Bool>("avoidance_active", 1, true);

	/***创建底盘运动话题订阅者***/
	ros::Subscriber vel_sub = node.subscribe("cmd_vel_ori", 1, cmd_vel_ori_Callback);

  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe("object_tracker/current_position", 10, current_position_Callback);

	private_nh.param<double>("safe_distence", safe_distence,0.5);
	private_nh.param<double>("danger_distence", danger_distence,0.2);
	private_nh.param<double>("danger_angular", danger_angular,0.785);
	private_nh.param<double>("avoidance_kv", avoidance_kv,0.2);
	private_nh.param<double>("avoidance_kw", avoidance_kw,0.3);
	private_nh.param<double>("max_vel_x", max_vel_x,1.5);
	private_nh.param<double>("max_vel_y", max_vel_y,1.5);
	private_nh.param<double>("max_linear_vel", max_linear_vel,1.5);
	private_nh.param<double>("min_vel_x", min_vel_x,0.05);
	private_nh.param<double>("max_vel_theta", max_vel_theta,1.5);
	private_nh.param<double>("min_vel_theta", min_vel_theta,0.05);
	private_nh.param<bool>("holonomic", holonomic,false);
	private_nh.param<double>("control_rate", control_rate,20.0);
	private_nh.param<double>("obstacle_timeout", obstacle_timeout,0.5);
	obstacle_timeout = fmax(0.0, obstacle_timeout);

	double rate2 = fmax(1.0, control_rate);
	ros::Rate loopRate2(rate2);

 
	while(ros::ok())
	{
		ros::spinOnce();
		if (!obstacle_received ||
			(obstacle_timeout > 0.0 &&
			 (ros::Time::now() - last_obstacle_time).toSec() > obstacle_timeout))
		{
			distance1 = 100.0;
			obstacle_received = false;
			ROS_WARN_THROTTLE(2.0, "Obstacle data unavailable or stale; disabling avoidance correction");
		}
		avoidance_kw = fabs(avoidance_kw);	// 角速度避障系数 先设置成正数
		cmd_vel_msg = cmd_vel_data;
		std_msgs::Bool avoidance_state;
		avoidance_state.data = obstacle_received && distance1 < safe_distence;
		if (holonomic)
		{
			// 麦轮无需先转动车头：直接在底盘坐标系叠加远离障碍物的二维速度。
			// dis_angleX: 前方为 0，右侧为正；ROS linear.y 左侧为正。
			if (distance1 < safe_distence)
			{
				const double distance_span = fmax(0.001, safe_distence - danger_distence);
				const double proximity = fmax(0.0, fmin(1.0,
					(safe_distence - distance1) / distance_span));
				const double repulsion = avoidance_kv * proximity;
				if (distance1 <= danger_distence)
				{
					// 危险区优先脱离障碍物，暂时放弃编队平移和航向跟随。
					cmd_vel_msg.linear.x = -avoidance_kv * cos(dis_angleX);
					cmd_vel_msg.linear.y =  avoidance_kv * sin(dis_angleX);
					cmd_vel_msg.angular.z = 0.0;
				}
				else
				{
					cmd_vel_msg.linear.x -= repulsion * cos(dis_angleX);
					cmd_vel_msg.linear.y += repulsion * sin(dis_angleX);
				}
			}
		}
		else if(distance1<safe_distence && distance1>danger_distence)		//障碍物在安全距离和危险距离时，调整速度角度避让障碍物
		{
			printf("distance1= %f\n",distance1);
			cmd_vel_msg.linear.x = cmd_vel_data.linear.x - fabs(cmd_vel_data.linear.x)*avoidance_kv*cos(dis_angleX)/distance1;//原始速度，减去一个后退的速度
			if(fabs(cmd_vel_msg.linear.x)>fabs(cmd_vel_data.linear.x) && cmd_vel_msg.linear.x*cmd_vel_data.linear.x>0)cmd_vel_msg.linear.x=cmd_vel_data.linear.x;//禁止小车加速
			if(dis_angleX<0)avoidance_kw=-avoidance_kw;											//车左右边的障碍物避障，车头调转方向不一致
			cmd_vel_msg.angular.z = cmd_vel_data.angular.z + avoidance_kw*cos(dis_angleX)/distance1;

		}
		else if(!holonomic && distance1<danger_distence)				//障碍物在危险距离之内时，以远离障碍物为主
		{
			printf("distance1= %f\n",distance1);
			cmd_vel_msg.linear.x =  - avoidance_kv*cos(dis_angleX);
			if(dis_angleX<0)avoidance_kw=-avoidance_kw;
			cmd_vel_msg.angular.z = avoidance_kw*cos(dis_angleX);
		}

		//速度限制
		if(cmd_vel_msg.linear.x > max_vel_x)
			cmd_vel_msg.linear.x=max_vel_x;
		else if(cmd_vel_msg.linear.x < -max_vel_x)
			cmd_vel_msg.linear.x=-max_vel_x;
		if(fabs(cmd_vel_msg.linear.x) < min_vel_x)
			cmd_vel_msg.linear.x=0;
		if(cmd_vel_msg.linear.y > max_vel_y)
			cmd_vel_msg.linear.y=max_vel_y;
		else if(cmd_vel_msg.linear.y < -max_vel_y)
			cmd_vel_msg.linear.y=-max_vel_y;
		if(fabs(cmd_vel_msg.linear.y) < min_vel_x)
			cmd_vel_msg.linear.y=0;
		const double linear_speed = hypot(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y);
		if(max_linear_vel > 0.0 && linear_speed > max_linear_vel)
		{
			const double scale = max_linear_vel / linear_speed;
			cmd_vel_msg.linear.x *= scale;
			cmd_vel_msg.linear.y *= scale;
		}
		if(cmd_vel_msg.angular.z > max_vel_theta)
			cmd_vel_msg.angular.z=max_vel_theta;
		else if(cmd_vel_msg.angular.z < -max_vel_theta)
			cmd_vel_msg.angular.z=-max_vel_theta;
		if(fabs(cmd_vel_msg.angular.z) < min_vel_theta)
			cmd_vel_msg.angular.z=0;

		cmd_vel_Pub.publish(cmd_vel_msg);
		avoidance_state_pub.publish(avoidance_state);
		ros::spinOnce();
		loopRate2.sleep();
	} 

	return 0;
}
