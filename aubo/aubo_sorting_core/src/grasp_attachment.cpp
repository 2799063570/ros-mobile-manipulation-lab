#include <aubo_sorting_core/color_sorting_task.hpp>
#include <sstream>

namespace aubo_sorting_core
{

void ColorSortingTask::graspStatusCallback(const std_msgs::StringConstPtr& message)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  grasp_status_ = message->data;// 抓取吸附的状态
  ++grasp_status_sequence_;
}

bool ColorSortingTask::waitForGraspPlugin()
{
  if (!use_grasp_attachment_) // 是否使用抓取吸附插件
    return true;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(grasp_attachment_timeout_);
  ros::WallRate rate(20.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status = grasp_status_;
    }
    if (status == "ready" || status.compare(0, 9, "detached:") == 0)
    {
      ROS_INFO("Gazebo grasp attachment plugin is ready");
      return true;
    }
    if (status.compare(0, 6, "error:") == 0)
    {
      ROS_ERROR_STREAM("Gazebo grasp plugin reported: " << status);
      return false;
    }
    rate.sleep();
  }
  ROS_ERROR_STREAM("No status received from Gazebo grasp plugin on " << grasp_status_topic_);
  return false;
}

bool ColorSortingTask::setGraspAttachment(const std::string& model_name, bool attach)
{
  // 通过话题通信请求抓取吸附插件进行吸附或释放物体
  // 通过判断序列号和状态来判断是否执行成功
  // 成功后注意修改attached_model_ 当前的吸附状态
  if (!use_grasp_attachment_)
    return true;
  const std::string expected = (attach ? "attached:" : "detached:") + model_name;
  std::uint64_t initial_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    initial_sequence = grasp_status_sequence_;
    // A timed-out attach may still have reached Gazebo; keep ownership for cleanup.
    if (attach)
      attached_model_ = model_name;
  }
  std_msgs::String command;
  command.data = model_name;
  (attach ? grasp_attach_publisher_ : grasp_detach_publisher_).publish(command);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(grasp_attachment_timeout_);
  ros::WallRate rate(50.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    std::string status;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status = grasp_status_;// 获取当前抓取状态
      sequence = grasp_status_sequence_;// 获取当前抓取状态序列号
    }
    const bool nearest_ack = attach && model_name.compare(0, 8, "nearest:") == 0 &&
        status.compare(0, 9, "attached:") == 0 &&
        (status.substr(9) == model_name.substr(8) ||
         status.substr(9).compare(0, model_name.size()-8+1, model_name.substr(8) + "_") == 0);
    if (sequence > initial_sequence && (status == expected || nearest_ack))
    {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        attached_model_ = attach ? (nearest_ack ? status.substr(9) : model_name) : std::string();
      }
      ROS_INFO_STREAM("Gazebo grasp status: " << status);
      return true;
    }
    if (sequence > initial_sequence && status.compare(0, 6, "error:") == 0)
    {
      ROS_ERROR_STREAM("Gazebo grasp plugin reported: " << status);
      return false;
    }
    rate.sleep();
  }
  ROS_ERROR_STREAM("Timed out waiting for Gazebo grasp status '" << expected << "'");
  return false;
}

void ColorSortingTask::releaseAttachedObjectNoWait()
{
  // 不用消息序列号和状态来判断是否执行成功 直接发送释放命令
  std::string model;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    model.swap(attached_model_);// 获取当前吸附的物体
  }
  if (use_grasp_attachment_ && !model.empty())
  {
    std_msgs::String command;
    command.data = model;
    grasp_detach_publisher_.publish(command);
  }
}

}  // namespace aubo_sorting_core
