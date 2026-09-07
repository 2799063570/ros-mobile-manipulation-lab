/*********************************************************************************************//**
* gripper_control.h
*

* May 2019
* Author:Ningning Yang

* *********************************************************************************************/



#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <inspire_gripper/set_openlimit.h>
#include <inspire_gripper/set_id.h>
#include <inspire_gripper/move_tgt.h>
#include <inspire_gripper/move_max.h>
#include <inspire_gripper/move_min.h>
#include <inspire_gripper/move_minhold.h>
#include <inspire_gripper/get_openlimit.h>
#include <inspire_gripper/get_copen.h>
#include <inspire_gripper/get_state.h>
#include <inspire_gripper/set_es.h>
#include <inspire_gripper/set_param.h>
#include <inspire_gripper/set_frsvd.h>

namespace inspire_gripper
{

class gripper_serial
{
public:


    gripper_serial(ros::NodeHandle *nh);

    ~gripper_serial();
    //设置函数的callback

    bool setOpenLimitCallback(inspire_gripper::set_openlimit::Request &req,
                              inspire_gripper::set_openlimit::Response &res);
    bool setIDCallback(inspire_gripper::set_id::Request &req,
                       inspire_gripper::set_id::Response &res);
    bool moveTgtCallback(inspire_gripper::move_tgt::Request &req,
                         inspire_gripper::move_tgt::Response &res);
    bool moveMaxCallback(inspire_gripper::move_max::Request &req,
                         inspire_gripper::move_max::Response &res);
    bool moveMinCallback(inspire_gripper::move_min::Request &req,
                         inspire_gripper::move_min::Response &res);
    bool moveMinHoldCallback(inspire_gripper::move_minhold::Request &req,
                             inspire_gripper::move_minhold::Response &res);
    bool getOpenlimitCallback(inspire_gripper::get_openlimit::Request &req,
                              inspire_gripper::get_openlimit::Response &res);
    bool getCopenCallback(inspire_gripper::get_copen::Request &req,
                          inspire_gripper::get_copen::Response &res);
    bool getStateCallback(inspire_gripper::get_state::Request &req,
                          inspire_gripper::get_state::Response &res);
    bool setEstopCallback(inspire_gripper::set_es::Request &req,
                          inspire_gripper::set_es::Response &res);
    bool setParamCallback(inspire_gripper::set_param::Request &req,
                          inspire_gripper::set_param::Response &res);
    bool setFrsvdCallback(inspire_gripper::set_frsvd::Request &req,
                          inspire_gripper::set_frsvd::Response &res);
    //void timerCallback(const ros::TimerEvent &event);

    //关节参数发布
    //ros::Publisher joint_pub;

    //TF更新周期
    //static const float TF_UPDATE_PERIOD = 0.5;


private:

    //设置开口限位（最大开口度和最小开口度）
    bool setOpenLimit(serial::Serial *port, int openmax, int openmin);
    //设置ID
    bool setID(serial::Serial *port, int id);
    //运动目标
    bool moveTgt(serial::Serial *port, int movetgt);
    //运动松开
    bool moveMax(serial::Serial *port, int speed);
    //运动抓取
    bool moveMin(serial::Serial *port, int speed, int power);
    //运动持续抓取
    bool moveMinHold(serial::Serial *port, int speed, int power);
    //读取开口限位
    void getOpenlimit(serial::Serial *port);
    //读取当前开口
    void getCopen(serial::Serial *port);
    //读取当前状态
    uint8_t getState(serial::Serial *port);
    int start(serial::Serial *port);
    //急停
    bool setEstop(serial::Serial *port);
    //参数固化
    bool setParam(serial::Serial *port);
    //清除故障
    bool setFrsvd(serial::Serial *port);
    /** \brief Set periodic position reading by GET_STATE(0x95) command */
    //void getPeriodicPositionUpdate(serial::Serial *port, float update_frequency);

    /** \brief Function to determine checksum*/
    uint16_t CRC16(uint16_t crc, uint16_t data);

    /** \brief Conversion from 4 bytes to float*/
    float IEEE_754_to_float(uint8_t *raw);

    /** \brief Conversion from float to 4 bytes*/
    void float_to_IEEE_754(float position, unsigned int *output_array);

    //Launch params
    int gripper_id_;
    int test_flags;
    std::string port_name_;
    int baudrate_;

    //Gripper state variables
    float act_position_;
    float openmax;
    float openmin;
    float curopen;


    int motion_state_{-1};
    uint8_t gripper_state_;
    //sensor_msgs::JointState gripper_joint_state_;

    //Serial variables
    serial::Serial *com_port_;

    //Consts
    //static const double MIN_GRIPPER_POS_LIMIT = 500;
    //static const double MAX_GRIPPER_POS_LIMIT = 7000;
    //static const double MIN_GRIPPER_VEL_LIMIT = 0;
    //static const double MAX_GRIPPER_VEL_LIMIT = 83;
    //static const double MIN_GRIPPER_ACC_LIMIT = 0;
    //static const double MAX_GRIPPER_ACC_LIMIT = 320;
    static constexpr double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
    static constexpr double INPUT_BUFFER_SIZE = 64;	   
    //static const int    URDF_SCALE_FACTOR = 2000;

};
}

#endif
