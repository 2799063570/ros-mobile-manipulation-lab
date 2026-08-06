#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

using namespace std;

//---机械臂关节目标控制角度---//
double control_joint_A = 0, control_joint_B = 0, control_joint_C = 0, control_joint_end = 0, control_joint_grasper = 0;
double control_joint_hand_left1 = 0, control_joint_hand_left2 = 0, control_joint_hand_right1 = 0, control_joint_hand_right2 = 0;
double curret_joint_end;

void Arm_2_joint_states_Callback(const sensor_msgs::JointState arm_joint);

/**************************************************************************
Function: Main function
Input   : none
Output  : 无
函数功能： 主函数
入口参数： 无
返回  值： 无
**************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc,argv,"arm_2_movit_Visualization"); //初始化节点
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    ros::NodeHandle NodeHandle; //创建节点句柄

    ros::Subscriber arm_2_joint_states_Sub; //机械臂关节状态订阅者
    //启动机械臂关节状态订阅者，用于获取由逆解求出来的关节状态以进行可视化
    arm_2_joint_states_Sub  = NodeHandle.subscribe("Arm_2_JointStates", 100, &Arm_2_joint_states_Callback);

    //开启机械臂和机械爪(习惯上统称机械臂)
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");

    arm.setGoalJointTolerance(0.001);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);
    hand.setGoalJointTolerance(0.001);
    hand.setMaxAccelerationScalingFactor(1);
    hand.setMaxVelocityScalingFactor(1);

    //控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    hand.setNamedTarget("home");
    arm.move();
    hand.move();
    sleep(1);

    while(ros::ok())
    {
        std::vector<double> arm_joint_positions(5); //机械臂关节控制变量
        std::vector<double> hand_joint_positions(4); //机械爪控制变量 

        arm_joint_positions[0] = control_joint_A;
        arm_joint_positions[1] = control_joint_B;
        arm_joint_positions[2] = control_joint_C;
        arm_joint_positions[3] = -control_joint_B-control_joint_C;
        arm_joint_positions[4] = control_joint_grasper;

        hand_joint_positions[0] = control_joint_hand_left1;
        hand_joint_positions[1] = control_joint_hand_left2;
        hand_joint_positions[2] = control_joint_hand_right1;
        hand_joint_positions[3] = control_joint_hand_right2;

        cout<<"arm.setJointValueTarget"<<endl;

        arm.setJointValueTarget(arm_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_success = ((arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS));
        if(arm_success)  arm.execute(arm_plan),sleep(1); //如果规划成功则执行

        hand.setJointValueTarget(hand_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
        bool hand_success = ((hand.plan(hand_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS));
        if(hand_success)  hand.execute(hand_plan),sleep(1); //如果规划成功则执行

        cout<<endl<<"Main"<<endl;
        cout<<"control_joint_end: "<<control_joint_end<<endl;

        cout<<"set complete."<<endl;
    }
    
    return 0;
}
/**************************************************************************
Function: The joint state callback function of the manipulator arm is used to obtain the joint state obtained from the inverse solution for MOVEIT visualization
Input   : Joint state topic
Output  : none
函数功能：机械臂关节状态回调函数，获取由逆解求出来的关节状态，用于Moveit可视化
入口参数：关节状态话题
返回  值：无
**************************************************************************/
void Arm_2_joint_states_Callback(const sensor_msgs::JointState arm_joint)
{
    control_joint_A           = arm_joint.position[0];
    //关节B目标角度(相对X轴)、关节C目标角度(相对关节B)转换为
    //关节B目标角度(相对关节B起始角度)、关节C目标角度(相对关节C起始角度)
    control_joint_B           = arm_joint.position[1];
    control_joint_C           = arm_joint.position[2];
    control_joint_hand_left1  = arm_joint.position[4];
    control_joint_hand_right1 = arm_joint.position[6];

    //使机械爪两指保持平行的姿态
    control_joint_hand_left2  = control_joint_hand_left1;
    control_joint_hand_right2 = control_joint_hand_right1;

    /*cout<<endl<<"Arm_2_joint_states_Callback"<<endl;
    cout<<"control_joint_hand_left2: "<<control_joint_hand_left2<<endl;
    cout<<"control_joint_hand_right2: "<<control_joint_hand_right2<<endl;*/
}


  
 