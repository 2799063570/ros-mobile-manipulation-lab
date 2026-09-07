#include <math.h> 
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h> 
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "inspire_gripper/set_id.h"
#include "inspire_gripper/set_openlimit.h"
#include "inspire_gripper/move_tgt.h"
//#include "inspire_gripper/hold_param.h"


using namespace std;

bool is_response(bool flag)
{
	sleep(0.2);
	if (flag)
	{
		ROS_INFO("请求正常处理,响应结果");
		return 0;
	}
	else
	{
		
		return 1;
	}


}

int main(int argc, char **argv)
{
	setlocale(LC_ALL,"");
	
	ros::init(argc, argv, "move_with_circle");//初始化节点 
	
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);//自旋线程
	spinner.start();

	moveit::planning_interface::MoveGroupInterface ur5("manipulator_i5");//选择规划组 manipulator_i5

	string eef_link = ur5.getEndEffector(); 		//获取末端连杆的名称
	std::string reference_frame = "base_link"; 		//设置参考坐标系名称
	ur5.setPoseReferenceFrame(reference_frame);		//设置参考坐标系名称

	ur5.allowReplanning(true);				//允许重新规划

	ur5.setGoalPositionTolerance(0.001);	//设置目标位置的允许距离误差
	ur5.setGoalOrientationTolerance(0.01);  //设置目标位置的允许角度误差
	ur5.setMaxAccelerationScalingFactor(0.8); //设置最大加速度
	ur5.setMaxVelocityScalingFactor(0.8);	  //设置最大速度
	
	ros::ServiceClient id_client = nh.serviceClient<inspire_gripper::set_id>("/inspire_gripper/set_id");	//创建客户端对象
	ros::ServiceClient openlimit_client = nh.serviceClient<inspire_gripper::set_openlimit>("/inspire_gripper/set_openlimit");	
	ros::ServiceClient move_client = nh.serviceClient<inspire_gripper::move_tgt>("/inspire_gripper/move_tgt");		
	//ros::ServiceClient hold_param_client = nh.serviceClient<inspire_gripper::hold_param>("/inspire_gripper/hold_param");	

	ros::service::waitForService("/inspire_gripper/set_id");//等待服务启动成功
	ros::service::waitForService("/inspire_gripper/set_openlimit");
	ros::service::waitForService("/inspire_gripper/move_tgt");
	//ros::service::waitForService("/inspire_gripper/hold_param");
	
	inspire_gripper::set_id inspire_id;					//服务通信 对应夹爪的ID
	inspire_gripper::set_openlimit openlimit;			//服务通信 对应夹爪的张合度
	inspire_gripper::move_tgt move_target;				//服务通信 对应夹爪移动的目标位置
	//inspire_gripper::move_minhold hold_param;			//服务通信 对应家住移动的参数：速度、力
	
	const int32_t loosen_opening = 800;//对应夹爪松开物块的夹角
	const int32_t clamp_opening = 100;//对应夹爪抓紧物块的夹角
	
	inspire_id.request.id = 1;	
	bool id_flag = id_client.call(inspire_id);//设置电动夹爪id
	if(is_response(id_flag))
		return 1;
	
	openlimit.request.openmax = 1000;
	openlimit.request.openmin = 10;
	bool openlimit_flag = openlimit_client.call(openlimit);//设置张合范围
	if(is_response(openlimit_flag))
		return 1;
	
	move_target.request.movetgt = loosen_opening;
	bool move_flag = move_client.call(move_target);
	if(is_response(move_flag))
		return 1;
	ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);
	sleep(1);
	
	double targetPose[6] = {0, 0, 3.14/2, 0.0, 3.14/2, 0};
        std::vector<double> joint_group_positions(6);
        joint_group_positions[0] = targetPose[0];
        joint_group_positions[1] = targetPose[1];
        joint_group_positions[2] = targetPose[2];
        joint_group_positions[3] = targetPose[3];
        joint_group_positions[4] = targetPose[4];
        joint_group_positions[5] = targetPose[5];

        ur5.setJointValueTarget(joint_group_positions);
        ur5.move();
        sleep(1);


	geometry_msgs::Pose target_pose = ur5.getCurrentPose(eef_link).pose;
	//获取当前位置

	target_pose.position.z -= 0.35;
	ur5.setPoseTarget(target_pose); 
	ur5.move();//向下移动去抓取
	sleep(2);
	
	
	move_target.request.movetgt = clamp_opening;
	move_flag = move_client.call(move_target);//夹爪闭合
	if(is_response(move_flag))
		return 1;
	ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);

	bool key_curise = 0;
	if(argc == 2)
	{
		key_curise = atoi(argv[1]);
		
	}
	if(key_curise)
	{
		ROS_INFO("使用单独点位控制");
		//设置物块目标位置2
		target_pose.position.z += 0.35;
		ur5.setPoseTarget(target_pose); 
		ur5.move();//避免碰撞
		sleep(1);

		//设置物块目标位置3
		target_pose.position.y += 0.5;
		ur5.setPoseTarget(target_pose); 
		ur5.move();//向左移动避免碰撞
		sleep(1);

		//设置物块目标位置3
		target_pose.position.z -= 0.35;
		ur5.setPoseTarget(target_pose); 
		ur5.move();//向左移动避免碰撞
		sleep(1);
	}
	else
	{
		ROS_INFO("使用连续轨迹控制");

		vector<geometry_msgs::Pose> waypoints; //创建容器变量 存储轨迹中离散化的点 
		waypoints.push_back(target_pose);//加入当前位置点到轨迹点容器中

		target_pose.position.z += 0.35;//先向上移动
 		waypoints.push_back(target_pose);
		target_pose.position.y += 0.5;//先向左移动
 		waypoints.push_back(target_pose);
		target_pose.position.z -= 0.35;//先向下移动
 		waypoints.push_back(target_pose);


		// 笛卡尔空间下的路径规划
		moveit_msgs::RobotTrajectory trajectory; 
		const double jump_threshold = 0.0;
		const double  eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;	//最大尝试规划次数
		int attempts = 0;	//已经尝试规划次数
		while(fraction < 1.0 && attempts < maxtries)
		{
			//生成笛卡尔路径 根据给定的起点和目标位置（在笛卡尔坐标系中），以及路径的约束条件（如步长、方向等），生成一条平滑的路径
			//waypoints：路径上的点(geometry_msgs/PoseStamped)
			//eef_step：起点和终点的容差，用于判断路径是否已经足够接近目标位置
			//jump_threshold：跳跃阈值
			fraction = ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); 
			attempts++;

			if(attempts %  10 == 0)
				ROS_INFO("Still trying after %d attempts...", attempts);
		}


		if(fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			// 生成机械臂的运动规划数据
			moveit::planning_interface::MoveGroupInterface::Plan plan; 
			plan.trajectory_ = trajectory;
						
			// 执行运动
			ur5.execute(plan); 
			sleep(1);

		}
		else
		{

			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);

		}
	}
	
	move_target.request.movetgt = loosen_opening;
	move_flag = move_client.call(move_target);//夹爪松开
	if(is_response(move_flag))
		return 1;
	ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);


	// 控制机械臂先回到初始化位置
	ur5.setNamedTarget("home"); 
	ur5.move();
	sleep(1);


	ros::shutdown(); 
	return 0;
}


