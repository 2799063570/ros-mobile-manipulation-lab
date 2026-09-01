#ifndef AUBO_ROS_CONTROL_VISUAL_SERVO_H
#define AUBO_ROS_CONTROL_VISUAL_SERVO_H

#include <aubo_ros_control/VisualServoConfig.h>
#include <aubo_ros_control/direct_sdk_backend.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aubo_ros_control {

/**
 * @brief AUBO i5 统一视觉伺服控制器。
 *
 * 类声明、回调接口和成员变量集中放在本头文件中，具体控制逻辑在
 * visual_servo.cpp 中实现。
 */
class VisualServo {
public:
  VisualServo();
  VisualServo(const VisualServo &) = delete;
  VisualServo &operator=(const VisualServo &) = delete;

  // 初始化是否成功；失败时节点应立即退出。
  bool valid() const;

  // SDK 后端运行期间是否仍保持健康；Gazebo 后端始终返回 true。
  bool healthy() const;

private:
  using JointPoint = visual_servo_internal::JointPoint;
  using CommandQueue = visual_servo_internal::CommandQueue;
  using DirectSdkBackend = visual_servo_internal::DirectSdkBackend;
  using ReconfigureServer = dynamic_reconfigure::Server<VisualServoConfig>;

  // 视觉伺服状态机：覆盖等待、跟踪、短时续行、搜索恢复和安全保持。
  enum class ServoState {
    DISABLED,
    WAITING,
    SEARCH_INITIAL,
    TRACKING,
    COAST,
    SEARCH_RECOVERY,
    HOLD
  };

  static const char *stateName(ServoState state);

  // 初始化配置、运动学链和后端发布通道。
  std::size_t readQueueCapacity();
  bool loadParameters();
  bool initializeKinematics();
  bool setupGazeboPublishers();
  void setupDynamicReconfigure();
  void reconfigureCallback(VisualServoConfig &config, uint32_t level);

  // ROS 输入、服务和真实机械臂状态回调。
  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &message);
  bool setEnabled(std_srvs::SetBool::Request &request,
                  std_srvs::SetBool::Response &response);
  bool reset(std_srvs::Trigger::Request &request,
             std_srvs::Trigger::Response &response);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &message);
  void setJointFeedback(const JointPoint &position);
  void sdkStateUpdate(const ros::TimerEvent &event);

  // 状态决策、PBVS 速度求解及周期控制输出。
  ServoState selectState(bool fresh_target, double target_age);
  Eigen::VectorXd trackingVelocity(const JointPoint &position,
                                   const geometry_msgs::Pose &target);
  Eigen::VectorXd desiredVelocity(ServoState state, const JointPoint &position,
                                  const geometry_msgs::Pose &target,
                                  double target_age);
  void transitionTo(ServoState next);
  void publishState();
  void controlLoop(const ros::TimerEvent &event);
  void gazeboOutput(const ros::TimerEvent &event);

  // ROS 通信对象、坐标变换缓存和有界指令队列。
  ros::NodeHandle nh_, private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  CommandQueue queue_;
  bool valid_{false};

  // 运行模式、坐标系、话题及控制器参数。
  std::string backend_, servo_mode_, base_link_, camera_link_, control_link_;
  std::string target_topic_, state_topic_, joint_states_topic_, loss_strategy_;
  std::vector<std::string> joint_names_, gazebo_topics_;
  double control_rate_{100.0}, output_rate_{200.0};
  double linear_gain_{0.8}, angular_gain_{0.5};
  double max_linear_velocity_{0.08}, max_angular_velocity_{0.2};
  double position_deadband_{0.004}, dls_lambda_{0.04},
      joint_limit_margin_{0.08};
  double target_timeout_{0.2}, coast_duration_{0.35}, coast_decay_time_{0.18};
  double minimum_safe_target_distance_{0.0};
  double search_timeout_{8.0}, open_posture_gain_{0.7},
      search_velocity_limit_{0.2};
  double feedback_blend_{0.02};
  bool use_orientation_control_{false}, initial_search_enabled_{false},
      enabled_{false};

  // 关节约束、搜索姿态、反馈状态和积分命令。
  JointPoint lower_limits_{}, upper_limits_{}, velocity_limits_{},
      acceleration_limits_;
  JointPoint open_posture_{}, initial_search_posture_{}, recovery_posture_;
  JointPoint feedback_position_{}, command_position_{}, command_velocity_;
  JointPoint backend_velocity_{}, last_output_;
  Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d desired_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd last_tracking_velocity_{
      Eigen::VectorXd::Zero(visual_servo_internal::kDof)};

  // KDL 正运动学、雅可比求解器以及真实机械臂 SDK 后端。
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<DirectSdkBackend> sdk_;

  // 多线程回调共享状态及对应互斥量。
  std::mutex target_mutex_, joint_mutex_, control_mutex_;
  geometry_msgs::Pose target_pose_;
  ros::Time last_target_time_, last_joint_time_;
  bool have_target_{false}, have_joint_state_{false},
      integrator_initialized_{false};
  bool have_ever_tracked_{false};
  std::atomic<bool> safety_stop_{false};
  ServoState state_{ServoState::DISABLED};

  // ROS 订阅、发布、服务和定时器句柄。
  ros::Subscriber target_sub_, joint_sub_;
  ros::Publisher state_pub_, legacy_state_pub_, sdk_joint_pub_;
  ros::ServiceServer enable_service_, reset_service_;
  std::vector<ros::Publisher> gazebo_publishers_;
  ros::Timer control_timer_, output_timer_, sdk_state_timer_;
  std::unique_ptr<ReconfigureServer> reconfigure_server_;
};

} // namespace aubo_ros_control

#endif // AUBO_ROS_CONTROL_VISUAL_SERVO_H
