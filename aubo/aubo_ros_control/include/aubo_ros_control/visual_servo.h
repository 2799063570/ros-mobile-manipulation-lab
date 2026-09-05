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
  VisualServo();// 构造函数：读取参数服务器中的参数，初始化运动学链、后端和 ROS 通信通道。
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

  // Task motions (observation/search/grasp) belong to the caller.
  using ServoState = visual_servo_internal::ServoState;

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
  void latchFault();
  Eigen::VectorXd trackingVelocity(const JointPoint &position,
                                   const geometry_msgs::Pose &target,
                                   Eigen::Vector3d *raw_position_error,
                                   Eigen::Vector3d *raw_angular_error);
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
  std::string target_topic_, state_topic_, joint_states_topic_;
  std::vector<std::string> joint_names_, gazebo_topics_;
  double control_rate_{100.0}, output_rate_{200.0};
  double linear_gain_{0.8}, angular_gain_{0.5};
  double max_linear_velocity_{0.08}, max_angular_velocity_{0.2};
  double position_deadband_{0.004}, dls_lambda_{0.04},
      joint_limit_margin_{0.08};// 死区阈值、DLS最小二乘阈值、关节位置限位余量
  double orientation_deadband_{0.02}, alignment_hold_time_{0.35},
      alignment_release_multiplier_{2.0};// 姿态死区阈值、对齐保持时间、迟滞因子
  double target_timeout_{0.2};
  double minimum_safe_target_distance_{0.0};
  double feedback_blend_{0.02};
  bool use_orientation_control_{false},
      enabled_{false};

  // 关节约束、反馈状态和积分命令。
  JointPoint lower_limits_{}, upper_limits_{}, velocity_limits_{},
      acceleration_limits_;
  JointPoint feedback_position_{}, command_position_{}, command_velocity_;
  JointPoint backend_velocity_{}, last_output_;
  Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d desired_rotation_{Eigen::Matrix3d::Identity()};
  // KDL 正运动学、雅可比求解器以及真实机械臂 SDK 后端。
  KDL::Chain chain_; // 机械臂运动学链 从 base_link_ 到 control_link_ 的链
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<DirectSdkBackend> sdk_;

  // 多线程回调共享状态及对应互斥量。
  std::mutex target_mutex_, joint_mutex_, control_mutex_;
  geometry_msgs::Pose target_pose_;
  ros::Time last_target_time_, last_joint_time_;
  bool have_target_{false}, have_joint_state_{false},
      integrator_initialized_{false};
  visual_servo_internal::AlignmentTracker alignment_;
  std::atomic<bool> safety_stop_{false};
  ServoState state_{ServoState::DISABLED};

  // ROS 订阅、发布、服务和定时器句柄。
  ros::Subscriber target_sub_, joint_sub_;
  ros::Publisher aligned_pub_, error_pub_, age_pub_, queue_pub_;
  ros::Publisher state_pub_, legacy_state_pub_, sdk_joint_pub_;
  ros::ServiceServer enable_service_, reset_service_;
  std::vector<ros::Publisher> gazebo_publishers_;
  ros::Timer control_timer_, output_timer_, sdk_state_timer_;
  std::unique_ptr<ReconfigureServer> reconfigure_server_;
};

} // namespace aubo_ros_control

#endif // AUBO_ROS_CONTROL_VISUAL_SERVO_H
