#include <math.h> 
#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group_interface.h> 
//#include <moveit/robot_trajectory/robot_trajectory.h>
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
        ROS_ERROR("请求处理失败....");
	return 1;
    }

}

int main(int argc, char **argv)
{
	setlocale(LC_ALL,"");
	
	ros::init(argc, argv, "gripper_control");//初始化节点 
	
	ros::NodeHandle nh;
	
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

	if(argc == 2)
	{
		ROS_INFO("使用输入参数控制电动夹爪的张合大小");
		move_target.request.movetgt = atoi(argv[1]);
		if(move_target.request.movetgt<=1000 && move_target.request.movetgt>=0)
		{	
			bool move_flag = move_client.call(move_target);
			if(is_response(move_flag))
				return 1;
			ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);
			sleep(1);
		}
		ROS_ERROR("输入参数有误....");
		return 1;
	}
	
	move_target.request.movetgt = loosen_opening;
	bool move_flag = move_client.call(move_target);
	if(is_response(move_flag))
		return 1;
	ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);
	sleep(2);

	move_target.request.movetgt = clamp_opening;
	move_flag = move_client.call(move_target);
	if(is_response(move_flag))
		return 1;
	ROS_INFO("电动夹爪移动到%d位置",move_target.request.movetgt);

	ros::shutdown(); 

	return 0;
}


