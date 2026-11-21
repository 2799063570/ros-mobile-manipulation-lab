#include <aubo_ros_control/motion_hold_fence.h>
#include <iostream>

int main()
{
    aubo_ros_control::MotionHoldFence fence;
    fence.acceptedBatch(7, 3, 5);
    fence.acceptedBatch(7, 63, 63);
    if (fence.confirmedAfterDiagnosis(6, 6) != 0)
    {
        std::cerr << "67 six-axis holds are required to exceed 400 scalars\n";
        return 1;
    }
    fence.acceptedBatch(7, 1, 1);
    if (fence.confirmedAfterDiagnosis(6, 6) != 7 ||
        fence.confirmedAfterDiagnosis(0, 6) != 0 ||
        fence.confirmedAfterDiagnosis(401, 6) != 0)
    {
        std::cerr << "The fence requires both replacement holds and a valid MAC watermark\n";
        return 1;
    }
    fence.acceptedBatch(8, 1, 1);
    if (fence.confirmedAfterDiagnosis(300, 6) != 0)
    {
        std::cerr << "A new switch cannot inherit an earlier fence\n";
        return 1;
    }
    fence.acceptedBatch(8, 64, 64);
    fence.acceptedBatch(8, 2, 4);
    if (fence.confirmedAfterDiagnosis(300, 6) != 0)
    {
        std::cerr << "A mixed batch interrupts the consecutive hold suffix\n";
        return 1;
    }
}
