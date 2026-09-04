#include <aubo_sorting_core/color_sorting_task.hpp>

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_sorting_task_cpp");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  try
  {
    aubo_sorting_core::ColorSortingTask task(nh, private_nh);
    task.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Color sorting task failed: " << exception.what());
    return 1;
  }
  return 0;
}
