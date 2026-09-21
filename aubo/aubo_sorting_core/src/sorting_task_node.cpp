#include <aubo_sorting_core/sorting_task.hpp>

#include <ros/ros.h>

#include <exception>

#ifndef AUBO_SORTING_NODE_NAME
#define AUBO_SORTING_NODE_NAME "sorting_task_cpp"
#endif

int main(int argc, char** argv)
{
  ros::init(argc, argv, AUBO_SORTING_NODE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  try
  {
    aubo_sorting_core::SortingTask task(nh, private_nh);
    task.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Sorting task failed: " << exception.what());
    return 1;
  }
  return 0;
}
