#include <gripper_control.h>

int
main(int argc, char *argv[])
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;

    //Create gripper object instance
    inspire_gripper::gripper_serial gripper(&nh);

    //Initialize user interface
    ros::ServiceServer set_openlimit_service = nh.advertiseService("inspire_gripper/set_openlimit", &inspire_gripper::gripper_serial::setOpenLimitCallback, &gripper);

    ros::ServiceServer set_id_service = nh.advertiseService("inspire_gripper/set_id", &inspire_gripper::gripper_serial::setIDCallback, &gripper);

    ros::ServiceServer move_tgt_service = nh.advertiseService("inspire_gripper/move_tgt", &inspire_gripper::gripper_serial::moveTgtCallback, &gripper);

    ros::ServiceServer move_max_service = nh.advertiseService("inspire_gripper/move_max", &inspire_gripper::gripper_serial::moveMaxCallback, &gripper);

    ros::ServiceServer move_min_service = nh.advertiseService("inspire_gripper/move_min", &inspire_gripper::gripper_serial::moveMinCallback, &gripper);

    ros::ServiceServer move_minhold_service = nh.advertiseService("inspire_gripper/move_minhold", &inspire_gripper::gripper_serial::moveMinHoldCallback, &gripper);

    ros::ServiceServer get_openlimit_service = nh.advertiseService("inspire_gripper/get_openlimit", &inspire_gripper::gripper_serial::getOpenlimitCallback, &gripper);

    ros::ServiceServer get_copen_service = nh.advertiseService("inspire_gripper/get_copen", &inspire_gripper::gripper_serial::getCopenCallback, &gripper);

    ros::ServiceServer get_state_service = nh.advertiseService("inspire_gripper/get_state", &inspire_gripper::gripper_serial::getStateCallback, &gripper);

    ros::ServiceServer set_es_service = nh.advertiseService("inspire_gripper/set_es", &inspire_gripper::gripper_serial::setEstopCallback, &gripper);

    ros::ServiceServer set_param_service = nh.advertiseService("inspire_gripper/set_param", &inspire_gripper::gripper_serial::setParamCallback, &gripper);

    ros::ServiceServer set_frsvd_service = nh.advertiseService("inspire_gripper/set_frsvd", &inspire_gripper::gripper_serial::setFrsvdCallback, &gripper);


    //ros::Timer timer = nh.createTimer(ros::Duration(gripper.TF_UPDATE_PERIOD), &inspire_gripper::hand_serial::timerCallback, &gripper);

    //gripper.joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    ros::spin();

    return(EXIT_SUCCESS);
}
