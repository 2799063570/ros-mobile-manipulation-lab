#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

#include <ros/package.h>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

using namespace aubo_robot_namespace;

namespace {
bool parseDouble(const char* text, double& value)
{
    char* end = nullptr;
    errno = 0;
    value = std::strtod(text, &end);
    return errno == 0 && end != text && *end == '\0' && std::isfinite(value);
}

bool waitReady(ServiceInterface& robot)
{
    for (int attempt = 0; attempt < 200; ++attempt) {
        RobotDiagnosis d{};
        if (robot.robotServiceGetRobotDiagnosisInfo(d) != InterfaceCallSuccCode)
            return false;
        if (d.armPowerStatus && !d.softEmergency && !d.remoteEmergency && !d.robotCollision)
            return true;
        usleep(100000);
    }
    return false;
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc != 9 || std::string(argv[1]) != "--execute") {
        std::cerr << "Guarded real-robot demo. Motion only occurs with the explicit flag.\n"
                  << "Usage: " << argv[0]
                  << " --execute ROBOT_IP q1 q2 q3 q4 q5 q6\n"
                  << "All joint targets are radians. Keep the E-stop within reach.\n";
        return 2;
    }

    const std::string package_path = ros::package::getPath("aubo_sdk");
    if (!package_path.empty() && chdir(package_path.c_str()) != 0)
        std::cerr << "Warning: unable to enter aubo_sdk runtime directory\n";

    std::array<double, ARM_DOF> target{};
    for (int i = 0; i < ARM_DOF; ++i) {
        if (!parseDouble(argv[i + 3], target[i])) {
            std::cerr << "Invalid joint target: " << argv[i + 3] << '\n';
            return 2;
        }
    }

    ServiceInterface robot;
    int ret = robot.robotServiceLogin(argv[2], 8899, "aubo", "123456");
    if (ret != InterfaceCallSuccCode) {
        std::cerr << "Login failed, SDK error: " << ret << '\n';
        return 1;
    }

    ToolDynamicsParam tool{};
    ROBOT_SERVICE_STATE startup_state;
    ret = robot.rootServiceRobotStartup(tool, 6, true, true, 1000, startup_state);
    if (ret == InterfaceCallSuccCode)
        ret = robot.robotServiceRobotHandShake(true);
    if (ret != InterfaceCallSuccCode || !waitReady(robot)) {
        std::cerr << "Robot startup/ready check failed, SDK error: " << ret << '\n';
        robot.robotServiceLogout();
        return 1;
    }

    std::cout << "Executing guarded joint move...\n";
    ret = robot.robotServiceJointMove(target.data(), true);
    robot.robotServiceLogout();
    if (ret != InterfaceCallSuccCode) {
        std::cerr << "Joint move failed, SDK error: " << ret << '\n';
        return 1;
    }
    std::cout << "Target reached; SDK session closed.\n";
    return 0;
}
