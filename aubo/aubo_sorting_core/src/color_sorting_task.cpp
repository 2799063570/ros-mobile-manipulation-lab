#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <inspire_gripper/move_max.h>
#include <inspire_gripper/move_min.h>
#include <inspire_gripper/set_es.h>
#include <inspire_gripper/get_state.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

namespace aubo_sorting_core
{
using detail::wallSleep;

ColorSortingTask::ColorSortingTask(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(std::move(nh)), private_nh_(std::move(private_nh)), tf_listener_(ros::Duration(30.0))
{
  loadParameters();
  robot_limits_valid_ = verifyLoadedUpperArmLimit();// 检证机械臂基座数第二个关节 

  arm_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));// 初始化MoveGroupInterface
  arm_->setEndEffectorLink(end_effector_link_);
  arm_->setPoseReferenceFrame(target_frame_);
  arm_->setPlanningTime(planning_time_);
  arm_->setNumPlanningAttempts(10);
  arm_->setMaxVelocityScalingFactor(velocity_scaling_);
  arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);
  planning_frame_ = arm_->getPlanningFrame();
  if (gripper_backend_ == "trajectory")
    gripper_client_.reset(new GripperClient(gripper_action_name_, true));
  else
  {
    inspire_open_client_ = nh_.serviceClient<inspire_gripper::move_max>("/inspire_gripper/move_max");
    inspire_close_client_ = nh_.serviceClient<inspire_gripper::move_min>("/inspire_gripper/move_min");
    inspire_state_client_ = nh_.serviceClient<inspire_gripper::get_state>("/inspire_gripper/get_state");
    inspire_stop_client_ = nh_.serviceClient<inspire_gripper::set_es>("/inspire_gripper/set_es");
  }

  state_publisher_ = nh_.advertise<std_msgs::String>("/sorting/state", 1, true);// 状态机
  detection_summary_publisher_ = nh_.advertise<std_msgs::String>("/sorting/detection_summary", 1, true);// 检测结果汇总 例如：红1,绿0,蓝1
  base_lock_publisher_ = nh_.advertise<std_msgs::Bool>(base_lock_topic_, 1, true);// 机械臂是否在执行任务 移动底盘是否需要被锁住
  failure_publisher_ = nh_.advertise<std_msgs::String>(failure_topic_, 1, true);// 任务失败原因
  target_cache_publisher_ = nh_.advertise<std_msgs::String>("/sorting/target_cache", 1, true);// 目标的信息目标颜色、平均位置、观测次数、置信度、离散程度、目标年龄以及是否已抓取
  grasp_attach_publisher_ = nh_.advertise<std_msgs::String>(grasp_attach_topic_, 1);// 将目标吸附到夹爪上
  grasp_detach_publisher_ = nh_.advertise<std_msgs::String>(grasp_detach_topic_, 1);// 从夹爪上分离目标

  detection_subscriber_ = nh_.subscribe(detections_topic_, 1, &ColorSortingTask::detectionCallback, this);
  grasp_status_subscriber_ = nh_.subscribe(grasp_status_topic_, 5,
                                           &ColorSortingTask::graspStatusCallback, this);// 抓取状态的反馈
  workspace_subscriber_ = nh_.subscribe(workspace_update_topic_, 1,
                                        &ColorSortingTask::workspaceUpdateCallback, this);// 动态接收新的工作区配置 例如换工作台、换桌子位置、换放置区域、换物体模型名称
  if (require_octomap_)
  {
    cloud_subscriber_ = nh_.subscribe(point_cloud_topic_, 1, &ColorSortingTask::cloudCallback, this);// 订阅点云话题
    planning_scene_subscriber_ = nh_.subscribe(planning_scene_topic_, 5,
                                               &ColorSortingTask::planningSceneCallback, this);// 订阅规划场景话题
    clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_octomap_service_);// 初始化清除Octomap服务客户端
  }

  services_.push_back(nh_.advertiseService("/sorting/move_to_observation",
                                            &ColorSortingTask::observeService, this));// 请求机械臂移动到观测位置
  services_.push_back(nh_.advertiseService("/sorting/start", &ColorSortingTask::startService, this));// 开始执行分拣任务请求
  services_.push_back(nh_.advertiseService("/sorting/stop", &ColorSortingTask::stopService, this));// 停止执行分拣任务请求
  services_.push_back(nh_.advertiseService("/sorting/open_gripper", &ColorSortingTask::openService, this));// 打开夹爪请求
  services_.push_back(nh_.advertiseService("/sorting/prepare_work",
                                            &ColorSortingTask::prepareWorkService, this));//让机械臂移动跑动模式
  services_.push_back(nh_.advertiseService("/sorting/home", &ColorSortingTask::homeService, this));// 请求机械臂移动到home
  services_.push_back(nh_.advertiseService("/sorting/configure_workspace",
                                            &ColorSortingTask::configureWorkspaceService, this));// 配置工作区请求

  std_msgs::Bool unlocked;
  unlocked.data = false;
  base_lock_publisher_.publish(unlocked);
  std_msgs::String empty_failure;
  failure_publisher_.publish(empty_failure);
  publishTargetCache();// 发布目前检测到的结果
  publishState(State::INITIALIZING, "waiting for Gazebo controllers");
}

ColorSortingTask::~ColorSortingTask()
{
  detection_subscriber_.shutdown();
  target_worker_shutdown_.store(true);
  queue_condition_.notify_all();
  stop_requested_.store(true);
  if (arm_)
    arm_->stop();
  if (gripper_client_)
    gripper_client_->cancelAllGoals();
  if (initialization_thread_.joinable())
    initialization_thread_.join();
  if (operation_thread_.joinable())
    operation_thread_.join();
  if (target_thread_.joinable())
    target_thread_.join();
}

void ColorSortingTask::start()
{
  if (continuous_sorting_)
    target_thread_ = std::thread(&ColorSortingTask::targetWorker, this);
  initialization_thread_ = std::thread(&ColorSortingTask::initialize, this);
}

const char* ColorSortingTask::stateName(State state)
{
  switch (state)
  {
    case State::DETECTING: return "DETECTING";
    case State::ERROR: return "ERROR";
    case State::HOMING: return "HOMING";
    case State::IDLE: return "IDLE";
    case State::INITIALIZING: return "INITIALIZING";
    case State::OBSERVING: return "OBSERVING";
    case State::OPENING: return "OPENING";
    case State::PICKING: return "PICKING";
    case State::PREPARING: return "PREPARING";
    case State::READY: return "READY";
    case State::SORTING: return "SORTING";
    case State::STOPPED: return "STOPPED";
  }
  return "ERROR";
}

void ColorSortingTask::publishState(State state, const std::string& detail)
{
  // 负责更新状态机的状态为state并通过话题发布
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state_ = state;
  }
  std_msgs::String message;
  message.data = stateName(state);
  if (!detail.empty())
    message.data += " | " + detail;
  state_publisher_.publish(message);
  ROS_INFO_STREAM("Sorting state: " << message.data);
}

void ColorSortingTask::setFailure(const std::string& category, const std::string& detail)
{
  std_msgs::String message;
  message.data = category + " | " + detail;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_failure_ = message.data;// 保存最近的失败信息  失败类型|失败详情
  }
  failure_publisher_.publish(message);
  ROS_ERROR_STREAM("Sorting failure: " << message.data);
}

void ColorSortingTask::initialize()
{
  // Resolve configured names against the actually loaded SRDF, before accepting work.
  const auto names = arm_->getNamedTargets();
  for (const auto& target : {work_ready_named_target_, observation_named_target_, finish_named_target_})
  {
    if (!target.empty() && std::find(names.begin(), names.end(), target) == names.end())
    {
      busy_.store(false);
      setFailure("CONFIGURATION_FAILED", "SRDF group '" + group_name_ + "' has no named target '" + target + "'");
      publishState(State::ERROR, "configured named target missing from loaded SRDF");
      return;
    }
  }
  if (!robot_limits_valid_) // 检证机械臂基座数第二个关节 是否在范围[-60, 60]内
  {
    busy_.store(false);
    publishState(State::ERROR, "loaded upperArm_joint limit is not +/-60 deg");
    return;
  }
  if (!waitForGraspPlugin()) // 等待抓取吸附插件加载完成
  {
    busy_.store(false);
    publishState(State::ERROR, "Gazebo grasp plugin unavailable");
    return;
  }
  publishState(State::INITIALIZING, "waiting for gripper action");
  if (gripper_client_ ? !gripper_client_->waitForServer(ros::Duration(gripper_server_timeout_)) :
      !(inspire_open_client_.waitForExistence(ros::Duration(gripper_server_timeout_)) &&
        inspire_close_client_.waitForExistence(ros::Duration(gripper_server_timeout_)) &&
        inspire_stop_client_.waitForExistence(ros::Duration(gripper_server_timeout_)) &&
        inspire_state_client_.waitForExistence(ros::Duration(gripper_server_timeout_))))// 等待夹爪动作服务器启动
  {
    busy_.store(false);
    publishState(State::ERROR, "gripper action server unavailable");
    return;
  }
  if (!addTableCollision()) // 添加桌子到规划场景中
  {
    busy_.store(false);
    publishState(State::ERROR, "sorting table missing from planning scene");
    return;
  }
  if (!refreshOctomap()) // 刷新octomap
  {
    busy_.store(false);
    publishState(State::ERROR, "RGB-D cloud or MoveIt OctoMap unavailable");
    return;
  }
  initialized_.store(true);// 完成初始化
  busy_.store(false);
  publishState(State::IDLE, "controllers ready");
  if (auto_move_to_observation_)
    startOperation(State::OBSERVING, std::bind(&ColorSortingTask::initialObservationOperation, this));
  else if (auto_start_)
    startOperation(State::SORTING, std::bind(&ColorSortingTask::sortingOperation, this));
}

std::pair<bool, std::string> ColorSortingTask::startOperation(
    State state, const std::function<bool()>& operation)
{
  // 启动一个操作 在非busy_下 
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (busy_.load())
      return std::make_pair(false, "another operation is running");
    if (!initialized_.load())
      return std::make_pair(false, "sorting node is not initialized");
    busy_.store(true);
    stop_requested_.store(false);// 清除停止请求标志
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      last_failure_.clear();// 清除故障信息
    }
    std_msgs::String empty;
    failure_publisher_.publish(empty);// 发布空字符串 清除故障信息
    std_msgs::Bool locked;
    locked.data = true;
    base_lock_publisher_.publish(locked);// 发布锁定信号 通知底盘锁定
  }
  // 如果操作线程已经存在并且可连接，则等待其完成
  if (operation_thread_.joinable())
    operation_thread_.join();
  // 启动一个新的线程来执行操作 捕获状态+操作函数
  operation_thread_ = std::thread([this, state, operation]() {
    bool success = false;
    publishState(state);
    try
    {
      success = operation();
    }
    catch (const std::exception& exception)
    {
      ROS_ERROR_STREAM("Sorting operation failed: " << exception.what());
      publishState(State::ERROR, exception.what());
    }
    {
      std::lock_guard<std::mutex> lock(operation_mutex_);
      busy_.store(false);// 操作完成，清除busy_标志
    }
    std_msgs::Bool unlocked;
    unlocked.data = false;
    base_lock_publisher_.publish(unlocked);// 发布解锁信号 通知底盘解锁
    if (stop_requested_.load())
    {
      observation_ready_.store(false);
      publishState(State::STOPPED, "operation cancelled");
    }
    else if (success)
      publishState(State::READY, "waiting for panel command");
    else
      publishState(State::ERROR, "operation failed");
  });
  return std::make_pair(true, "command accepted");
}

bool ColorSortingTask::observeService(std_srvs::Trigger::Request&,
                                      std_srvs::Trigger::Response& response)
{
  const auto result = startOperation(State::OBSERVING,
      std::bind(&ColorSortingTask::observationOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::startService(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& response)
{
  if (!observation_ready_.load())
  {
    response.success = false;
    response.message = "move to observation pose and confirm detections first";
    return true;
  }
  const auto result = startOperation(State::SORTING, std::bind(&ColorSortingTask::sortingOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::stopService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  std::lock_guard<std::mutex> lock(operation_mutex_);
  stop_requested_.store(true);
  if (gripper_client_)
    gripper_client_->cancelAllGoals();
  else
  {
    inspire_gripper::set_es stop;
    if (!inspire_stop_client_.call(stop) || !stop.response.setes_accepted)
    {
      arm_->stop();
      response.success = false;
      response.message = "Inspire stop was not acknowledged";
      return true;
    }
  }
  arm_->stop();
  if (!busy_.load() && initialized_.load())
  {
    // Idle stop must also provide fresh acknowledgement for mission recovery.
    std_msgs::Bool unlocked;
    unlocked.data = false;
    base_lock_publisher_.publish(unlocked);
    publishState(State::STOPPED, "idle stop confirmed");
  }
  response.success = true;
  response.message = "stop requested";
  return true;
}

bool ColorSortingTask::openService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  const auto result = startOperation(State::OPENING, std::bind(&ColorSortingTask::openOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::prepareWorkService(std_srvs::Trigger::Request&,
                                          std_srvs::Trigger::Response& response)
{
  const auto result = startOperation(State::PREPARING,
      std::bind(&ColorSortingTask::prepareWorkOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::homeService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  const auto result = startOperation(State::HOMING, std::bind(&ColorSortingTask::homeOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::observationOperation()
{
  if (continuous_sorting_)
    resetInstanceQueue();
  if (!observation())
    return false;
  if (!continuous_sorting_ && verify_observation_detections_ && !verifyVisibleColors()) // Legacy per-class verification.
  {
    observation_ready_.store(false);
    return false;
  }
  return true;
}

bool ColorSortingTask::initialObservationOperation()
{
  if (!observationOperation())
    return false;
  return !auto_start_ || sortingOperation();
}

bool ColorSortingTask::openOperation()
{
  if (!commandGripper(gripper_open_))
    return false;
  std::string model;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    model = attached_model_;
  }
  return model.empty() || setGraspAttachment(model, false);
}

bool ColorSortingTask::homeOperation()
{
  observation_ready_.store(false);
  return moveNamed(finish_named_target_);
}

bool ColorSortingTask::prepareWorkOperation()
{
  observation_ready_.store(false);
  return refreshOctomap() && addTableCollision() && moveNamed(work_ready_named_target_);
}

bool ColorSortingTask::observation()
{
  observation_ready_.store(false);
  if (!refreshOctomap() || !addTableCollision())  // 需要先更新octomap和场景碰撞模型
    return false;
  const bool success = observation_named_target_.empty() ?
      moveToPose(makePose(observation_pose_[0], observation_pose_[1], observation_pose_[2]),
                 "camera observation pose") :
      moveNamed(observation_named_target_);
  if (!success || !wallSleep(detection_settle_time_, stop_requested_))
    return false;
  observation_ready_.store(true);
  return true;
}

}  // namespace aubo_sorting_core
