#include <aubo_mobile_nav_sorting/navigation_sorting_mission.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_sorting_mission");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  try
  {
    aubo_mobile_nav_sorting::NavigationSortingMission mission(node_handle, private_node_handle);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL_STREAM("Could not start C++ navigation-sorting mission: " << error.what());
    return 1;
  }
  return 0;
}
