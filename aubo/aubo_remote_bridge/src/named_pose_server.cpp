/**
 * 功能：把HOME/ZERO请求转换成MoveGroup Action，回传接收、规划、执行、核对和终态。
 * 范围：仅支持独立Gazebo验证场景；所有运动接口固定在/remote_validation命名空间。
 * 线程：Action客户端不另开回调线程，与ros::spin共用队列，状态只在同一线程改变。
 * 关键：一次只有一个活动任务；服务立即返回应答，真正完成需要Action和实际关节共同确认。
 * 取消：cancelGoal只针对本节点的当前目标，终止未确认之前保持占用，拒绝新运动。
 */
#include "aubo_remote_bridge/posix_serial.hpp"
#include <aubo_remote_bridge/CommandStatus.h>
#include <aubo_remote_bridge/NamedPose.h>
#include <aubo_remote_bridge/RemoteCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/ModelStates.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/CollisionObject.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>

namespace abr = aubo_remote_bridge;
class NamedPoseServer {
 public:
  NamedPoseServer() : action_("/remote_validation/move_group", false),
      loader_("/remote_validation/robot_description"), model_(loader_.getModel()) {
    // 本节点只为指定Gazebo场景服务，接口名称刻意固定在独立命名空间。
    if (!model_ || !(group_ = model_->getJointModelGroup("aubo_i5"))) throw std::runtime_error("仿真机器人模型或aubo_i5规划组不可用");
    for (const auto& name : {std::string("home"), std::string("zero")}) {
      moveit::core::RobotState target(model_); target.setToDefaultValues();
      if (!target.setToDefaultValues(group_, name) || !target.satisfiesBounds(group_)) throw std::runtime_error("SRDF命名位置缺失或超出关节范围：" + name);
      std::vector<double> values;
      for (const auto& joint : abr::jointNames()) values.push_back(target.getVariablePosition(joint));
      targets_[name == "home" ? REMOTE_HOME : REMOTE_ZERO] = values;
    }
    result_pub_ = nh_.advertise<abr::CommandStatus>("/remote/command_result", 10, true);
    state_pub_ = nh_.advertise<abr::CommandStatus>("/remote/motion_state", 1, true);
    command_sub_ = nh_.subscribe("/remote/command", 1, &NamedPoseServer::command, this);
    joint_sub_ = nh_.subscribe("/remote_validation/aubo_i5/joint_states", 1, &NamedPoseServer::joints, this);
    model_sub_ = nh_.subscribe("/gazebo/model_states", 1, &NamedPoseServer::models, this);
    clock_sub_ = nh_.subscribe("/clock", 1, &NamedPoseServer::clock, this);
    joy_sub_ = nh_.subscribe("/remote/joy", 1, &NamedPoseServer::joy, this);
    link_sub_ = nh_.subscribe("/remote/connected", 1, &NamedPoseServer::link, this);
    valid_sub_ = nh_.subscribe("/remote/input_valid", 1, &NamedPoseServer::valid, this);
    service_ = nh_.advertiseService("/remote/request_named_pose", &NamedPoseServer::service, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(.05), &NamedPoseServer::tick, this);
    ROS_INFO("Gazebo命名位置请求节点已启动：home/zero；无任何AUBO SDK或真机控制入口");
  }
 private:
  using Client = actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>;
  bool simulationReady() const {
    // 仅有/use_sim_time不足以证明是本场景：还要看到专用模型、前进的/clock、真实仿真反馈和专用Action。
    bool sim = false; nh_.getParam("/use_sim_time", sim);
    const double now = abr::monotonicNow();
    return sim && model_present_ && now - model_time_ < .5 && now - clock_time_ < .5 && now - joint_time_ < .5 && action_.isServerConnected();
  }
  bool linkReady() const { return link_ && valid_ && abr::monotonicNow() - joy_time_ < .3; }
  void publishReply(const abr::Reply& reply, bool save = true) {
    if (save) ledger_.save(reply);
    abr::CommandStatus message; message.session = reply.request.session; message.request_id = reply.request.id;
    message.command = reply.request.command; message.state = reply.state; message.message = reply.message;
    result_pub_.publish(message);
  }
  bool submit(const abr::Request& request, bool require_link, std::string& reason) {
    // 服务和串口共用入口：先查幂等缓存，再做单任务占用与就绪检查，最后才允许sendGoal。
    abr::Reply previous;
    const auto lookup = ledger_.find(request, previous);
    if (lookup == abr::Ledger::Duplicate) { publishReply(previous, false); reason = "DUPLICATE: " + previous.message; return previous.state != REMOTE_REJECTED; }
    if (lookup == abr::Ledger::Conflict) { reason = "同一会话/编号不能更换命令"; publishReply({request, REMOTE_REJECTED, reason}, false); return false; }
    if (!request.id || request.command < REMOTE_HOME || request.command > REMOTE_CANCEL) {
      reason = "请求编号或命令无效"; publishReply({request, REMOTE_REJECTED, reason}); return false;
    }
    if (request.command == REMOTE_CANCEL) {
      if (has_cancel_) { reason = "已有取消请求等待终止确认"; publishReply({request, REMOTE_REJECTED, reason}); return false; }
      if (!active_) { reason = "当前没有活动任务"; publishReply({request, REMOTE_SUCCEEDED, reason}); return true; }
      cancel_request_ = request; has_cancel_ = true;
      publishReply({request, REMOTE_ACCEPTED, "已接收取消请求"});
      cancel("用户取消"); reason = "正在等待Action终止确认"; return true;
    }
    if (active_ || !simulationReady() || (require_link && !linkReady())) {
      reason = active_ ? "BUSY：前一任务或取消尚未终止" : "Gazebo/状态/串口输入尚未就绪";
      publishReply({request, REMOTE_REJECTED, reason}); return false;
    }
    active_ = true; canceled_ = false; verifying_ = false; has_cancel_ = false;
    active_request_ = request; requires_link_ = require_link;
    started_ = abr::monotonicNow(); state_ = REMOTE_PLANNING; detail_ = "规划命名位置";
    publishReply({request, REMOTE_ACCEPTED, "请求已接收，开始规划"});
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = "aubo_i5"; goal.request.allowed_planning_time = 5.; goal.request.num_planning_attempts = 3;
    goal.request.max_velocity_scaling_factor = .15; goal.request.max_acceleration_scaling_factor = .15;
    goal.request.start_state.is_diff = true;
    moveit_msgs::Constraints constraints;
    for (std::size_t i = 0; i < abr::jointNames().size(); ++i) {
      moveit_msgs::JointConstraint joint; joint.joint_name = abr::jointNames()[i]; joint.position = targets_.at(request.command)[i];
      joint.tolerance_above = .01; joint.tolerance_below = .01; joint.weight = 1.; constraints.joint_constraints.push_back(joint);
    }
    goal.request.goal_constraints.push_back(constraints);
    goal.planning_options.plan_only = false; goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    // 场景围挡位于工作区外；规划失败必须返回FAILED，不能绕过MoveIt直接发送关节角。
    addScene(goal.planning_options.planning_scene_diff);
    action_.sendGoal(goal,
      [this](const actionlib::SimpleClientGoalState& status, const moveit_msgs::MoveGroupResultConstPtr& result) { done(status, result); },
      [this]() { detail_ = "MoveIt已激活请求"; },
      [this](const moveit_msgs::MoveGroupFeedbackConstPtr& feedback) {
        if (canceled_ || verifying_) return;
        state_ = feedback->state == "MONITOR" || feedback->state == "EXECUTING" ? REMOTE_EXECUTING : REMOTE_PLANNING;
        detail_ = feedback->state;
      });
    reason = "ACCEPTED"; return true;
  }
  static void addScene(moveit_msgs::PlanningScene& scene) {
    const auto box = [](const std::string& name, double x, double y, double z, double sx, double sy, double sz) {
      moveit_msgs::CollisionObject object; object.id = name; object.header.frame_id = "world";
      shape_msgs::SolidPrimitive shape; shape.type = shape.BOX; shape.dimensions = {sx, sy, sz};
      geometry_msgs::Pose pose; pose.orientation.w = 1.; pose.position.x = x; pose.position.y = y; pose.position.z = z;
      object.primitives.push_back(shape); object.primitive_poses.push_back(pose); object.operation = object.ADD; return object;
    };
    scene.world.collision_objects.push_back(box("validation_floor", 0, 0, -.1, 4, 4, .1));
    scene.world.collision_objects.push_back(box("validation_fence_left", 0, 1.6, .1, 3.2, .08, .2));
    scene.world.collision_objects.push_back(box("validation_fence_right", 0, -1.6, .1, 3.2, .08, .2));
  }
  void cancel(const std::string& reason) {
    if (!active_ || canceled_) return;
    if (verifying_) { finish(REMOTE_CANCELED, "轨迹Action已经结束，取消后续到位核对：" + reason); return; }
    canceled_ = true; verifying_ = false; cancel_started_ = abr::monotonicNow(); state_ = REMOTE_CANCELING; detail_ = reason;
    // 只取消本节点发出的当前goal，不能cancelAllGoals误伤其他客户端。
    action_.cancelGoal();
  }
  void finish(uint8_t state, const std::string& reason) {
    state_ = state; detail_ = reason; active_ = false; verifying_ = false;
    publishReply({active_request_, state, reason});
    if (has_cancel_) { publishReply({cancel_request_, state == REMOTE_FAILED ? REMOTE_FAILED : REMOTE_SUCCEEDED, reason}); has_cancel_ = false; }
  }
  void done(const actionlib::SimpleClientGoalState& status, const moveit_msgs::MoveGroupResultConstPtr& result) {
    // 使用Action终态与MoveIt错误码双重判定；取消与成功同时发生时，不能把已完成动作伪报为已中止。
    if (!active_) return;
    if (status == actionlib::SimpleClientGoalState::LOST) {
      // LOST只代表客户端丢失目标状态，不证明执行器已经结束；保持占用，不能自动接下一个目标。
      canceled_ = true; verifying_ = false; cancel_started_ = abr::monotonicNow(); state_ = REMOTE_FAILED;
      detail_ = "Action连接失去终态，保持任务锁定"; publishReply({active_request_, REMOTE_FAILED, detail_}); return;
    }
    if (status == actionlib::SimpleClientGoalState::SUCCEEDED && result && result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
      // Action成功之后还要核对Gazebo真实JointState，不能把接收/规划成功当作已经到位。
      verifying_ = true; verified_since_ = -1; verify_started_ = abr::monotonicNow();
      verify_count_ = joint_count_; verify_last_sample_ = -1;
      state_ = REMOTE_VERIFYING; detail_ = "核对实际关节误差";
    } else if (canceled_ && (status == actionlib::SimpleClientGoalState::PREEMPTED || status == actionlib::SimpleClientGoalState::RECALLED)) finish(REMOTE_CANCELED, "Action已确认取消");
    else finish(REMOTE_FAILED, "MoveIt执行失败：" + status.toString());
  }
  void tick(const ros::WallTimerEvent&) {
    // 墙上时间定时器：Gazebo暂停时仍能请求取消，不被停止的ROS仿真时间冻结。
    const double now = abr::monotonicNow();
    if (active_) {
      if (!simulationReady() || (requires_link_ && !linkReady()) || now - started_ > 60.) cancel("状态/链路失效或任务超过60秒");
      if (canceled_ && !verifying_ && now - cancel_started_ > 2.) {
        // 取消未确认时仍保持active_，禁止下一任务；FAILED不表示已经停止。
        if (state_ != REMOTE_FAILED) publishReply({active_request_, REMOTE_FAILED, "取消尚未确认，保持任务锁定"});
        state_ = REMOTE_FAILED; detail_ = "终止状态尚未确认，保持任务锁定";
      }
      if (verifying_) {
        // 只在新的真实关节帧到达时累计确认，不能反复检查同一个缓存样本凑满200ms。
        if (verify_count_ != joint_count_) {
          verify_count_ = joint_count_;
          bool reached = now - joint_time_ < .5;
          const auto& target = targets_.at(active_request_.command);
          for (std::size_t i = 0; i < 6; ++i) reached = reached && std::abs(position_[i] - target[i]) <= .03;
          if (!reached || joint_time_ - verify_last_sample_ > .15) verified_since_ = -1;
          verify_last_sample_ = joint_time_;
          if (reached && verified_since_ < 0) verified_since_ = joint_time_;
          if (verified_since_ >= 0 && joint_time_ - verified_since_ >= .2)
            finish(REMOTE_SUCCEEDED, "实际关节连续200ms处于目标0.03rad范围内");
        }
        if (active_ && now - verify_started_ > 3.) finish(REMOTE_FAILED, "Action成功但实际关节未在期限内到位");
      }
    }
    abr::CommandStatus snapshot; snapshot.session = active_request_.session; snapshot.request_id = active_request_.id;
    snapshot.command = active_request_.command; snapshot.state = active_ || state_ != REMOTE_IDLE ? state_ : simulationReady() ? REMOTE_IDLE : REMOTE_UNKNOWN;
    snapshot.message = detail_; state_pub_.publish(snapshot);
  }
  void command(const abr::RemoteCommand::ConstPtr& message) {
    std::string reason; submit({message->session, message->request_id, message->command}, true, reason);
  }
  bool service(abr::NamedPose::Request& request, abr::NamedPose::Response& response) {
    // 无硬件验证通道不依赖串口心跳，但仍受同样的Gazebo专用模型/控制器检查约束。
    const uint8_t command = request.target == "home" ? REMOTE_HOME : request.target == "zero" ? REMOTE_ZERO : request.target == "cancel" ? REMOTE_CANCEL : 0;
    response.accepted = submit({request.session, request.request_id, command}, false, response.message); return true;
  }
  void models(const gazebo_msgs::ModelStates::ConstPtr& message) {
    model_present_ = std::find(message->name.begin(), message->name.end(), "aubo_remote_validation") != message->name.end(); model_time_ = abr::monotonicNow();
  }
  void clock(const rosgraph_msgs::Clock::ConstPtr& message) { if (message->clock > last_clock_) clock_time_ = abr::monotonicNow(); last_clock_ = message->clock; }
  void joy(const sensor_msgs::Joy::ConstPtr&) { joy_time_ = abr::monotonicNow(); }
  void link(const std_msgs::Bool::ConstPtr& message) { link_ = message->data; if (!link_ && active_ && requires_link_) cancel("串口离线"); }
  void valid(const std_msgs::Bool::ConstPtr& message) { valid_ = message->data; if (!valid_ && active_ && requires_link_) cancel("摇杆输入失效"); }
  void joints(const sensor_msgs::JointState::ConstPtr& message) {
    joint_time_ = -1e9;  // 非法新消息立即使旧样本失效，等待下一份完整的六关节反馈。
    if (message->name.size() != message->position.size() || std::set<std::string>(message->name.begin(), message->name.end()).size() != message->name.size()) return;
    const double age = (ros::Time::now() - message->header.stamp).toSec();
    if (!message->header.stamp.isZero() && (age < 0 || age > .5)) return;
    std::array<double, 6> next{};
    for (std::size_t i = 0; i < 6; ++i) {
      const auto found = std::find(message->name.begin(), message->name.end(), abr::jointNames()[i]);
      if (found == message->name.end()) return;
      next[i] = message->position[static_cast<std::size_t>(found - message->name.begin())]; if (!std::isfinite(next[i])) return;
    }
    position_ = next; joint_time_ = abr::monotonicNow(); ++joint_count_;
  }
  ros::NodeHandle nh_; Client action_;
  robot_model_loader::RobotModelLoader loader_; moveit::core::RobotModelPtr model_; const moveit::core::JointModelGroup* group_ = nullptr;
  ros::Publisher result_pub_, state_pub_; ros::Subscriber command_sub_, joint_sub_, model_sub_, clock_sub_, joy_sub_, link_sub_, valid_sub_;
  ros::ServiceServer service_; ros::WallTimer timer_; ros::Time last_clock_;
  std::map<uint8_t, std::vector<double>> targets_; std::array<double, 6> position_{};
  abr::Ledger ledger_; abr::Request active_request_, cancel_request_;
  bool active_ = false, canceled_ = false, verifying_ = false, requires_link_ = false, has_cancel_ = false;
  bool model_present_ = false, link_ = false, valid_ = false;
  double model_time_ = -1e9, clock_time_ = -1e9, joint_time_ = -1e9, joy_time_ = -1e9;
  double started_ = 0, cancel_started_ = 0, verify_started_ = 0, verified_since_ = -1;
  double verify_last_sample_ = -1; uint64_t joint_count_ = 0, verify_count_ = 0;
  uint8_t state_ = REMOTE_IDLE; std::string detail_ = "等待Gazebo和MoveIt就绪";
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "aubo_remote_named_pose");
  try { NamedPoseServer server; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("仿真请求节点启动失败：%s", error.what()); return 1; }
  return 0;
}
