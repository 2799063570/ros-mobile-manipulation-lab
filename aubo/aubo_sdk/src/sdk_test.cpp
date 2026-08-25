#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <unistd.h>

#include <ros/package.h>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

using namespace aubo_robot_namespace;

int main(int argc, char** argv)
{
    if (argc > 3) {
        std::cerr << "Usage: " << argv[0] << " [robot_ip] [port]\n";
        return 2;
    }

    const std::string package_path = ros::package::getPath("aubo_sdk");
    if (!package_path.empty() && chdir(package_path.c_str()) != 0)
        std::cerr << "Warning: unable to enter aubo_sdk runtime directory\n";

    const std::string host = argc >= 2 ? argv[1] : "192.168.1.2";
    const int port = argc >= 3 ? std::atoi(argv[2]) : 8899;
    ServiceInterface robot;

    std::cout << "Connecting to " << host << ':' << port << " ...\n";
    const int login_ret = robot.robotServiceLogin(host.c_str(), port, "aubo", "123456");
    if (login_ret != InterfaceCallSuccCode) {
        std::cerr << "Login failed, SDK error: " << login_ret << '\n';
        return 1;
    }

    bool real_robot = false;
    const int real_ret = robot.robotServiceGetIsRealRobotExist(real_robot);
    RobotDiagnosis diagnosis{};
    const int diagnosis_ret = robot.robotServiceGetRobotDiagnosisInfo(diagnosis);
    bool mac_connected = false;
    const int mac_ret = robot.robotServiceGetMacCommunicationStatus(mac_connected);
    wayPoint_S waypoint{};
    const int waypoint_ret = robot.robotServiceGetCurrentWaypointInfo(waypoint);

    std::cout << std::boolalpha
              << "real_robot=" << (real_ret == InterfaceCallSuccCode && real_robot) << '\n'
              << "mac_connected=" << (mac_ret == InterfaceCallSuccCode && mac_connected) << '\n';
    if (diagnosis_ret == InterfaceCallSuccCode) {
        std::cout << "power_on=" << diagnosis.armPowerStatus
                  << " brake=" << diagnosis.brakeStuats
                  << " soft_estop=" << diagnosis.softEmergency
                  << " remote_estop=" << diagnosis.remoteEmergency
                  << " collision=" << diagnosis.robotCollision << '\n';
    } else {
        std::cerr << "Diagnosis query failed, SDK error: " << diagnosis_ret << '\n';
    }

    if (waypoint_ret == InterfaceCallSuccCode) {
        std::cout << std::fixed << std::setprecision(6) << "joints_rad=";
        for (int i = 0; i < ARM_DOF; ++i)
            std::cout << (i == 0 ? "[" : ", ") << waypoint.jointpos[i];
        std::cout << "]\nposition_m=[" << waypoint.cartPos.position.x << ", "
                  << waypoint.cartPos.position.y << ", "
                  << waypoint.cartPos.position.z << "]\nquaternion_xyzw=["
                  << waypoint.orientation.x << ", " << waypoint.orientation.y << ", "
                  << waypoint.orientation.z << ", " << waypoint.orientation.w << "]\n";
    } else {
        std::cerr << "Waypoint query failed, SDK error: " << waypoint_ret << '\n';
    }

    robot.robotServiceLogout();
    return (diagnosis_ret == InterfaceCallSuccCode && waypoint_ret == InterfaceCallSuccCode) ? 0 : 1;
}
