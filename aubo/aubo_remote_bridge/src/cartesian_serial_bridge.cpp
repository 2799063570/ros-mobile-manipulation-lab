/** 串口仅调用/cartesian_jog接口；不连接SDK、不加载第二个运动执行器。
 * ROS回调与串口均在主线程；服务调用放在异步任务，确保取消和链路看门狗不被阻塞。
 */
#include <clocale>
#include "aubo_remote_bridge/jog_remote_core.hpp"
#include "aubo_remote_bridge/posix_serial.hpp"
#include "aubo_remote_bridge/serial_diagnostics.hpp"
#include <aubo_ros_control/MoveHomeAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <future>
#include <random>
#include <set>

namespace abr = aubo_remote_bridge;
namespace {
volatile std::sig_atomic_t stopping = 0;
void stopSignal(int) { stopping = 1; }
bool valid(const RemoteJog_Input& in) { return (in.flags & 3) == 3; }
bool neutral(const RemoteJog_Input& in) {
  return !(in.flags & REMOTE_JOG_DEADMAN) && !in.axis[0] && !in.axis[1] && !in.axis[2];
}
uint8_t controllerCode(const std::string& state) {
  if (state == "WAITING_FOR_FEEDBACK" || state == "WAITING_FOR_SDK") return REMOTE_JOG_WAITING;
  if (state == "DISABLED") return REMOTE_JOG_DISABLED;
  if (state == "ENABLED_IDLE") return REMOTE_JOG_IDLE;
  if (state == "JOGGING") return REMOTE_JOG_MOVING;
  if (state == "STOPPING") return REMOTE_JOG_STOPPING;
  if (state == "HOME_MOVING") return REMOTE_JOG_HOMING;
  if (state == "HOME_STOPPING") return REMOTE_JOG_HOME_STOPPING;
  if (state.find("FAULT:") == 0) return REMOTE_JOG_FAULT;
  return REMOTE_JOG_UNKNOWN;
}
uint16_t faultCode(const std::string& state) {
  if (state.find("FAULT:") != 0) return 0;
  const char* faults[] = {"feedback_timeout", "command_timeout", "invalid_or_stale_command",
    "singularity_stop", "collision_or_stopping_limit", "following_error", "stream_schedule_gap",
    "control_computation_overrun", "home_timeout", "hard_joint_limit", "sdk_feedback_failed",
    "invalid_feedback", "kinematics_failed", "invalid_joint_velocity"};
  for (std::size_t i = 0; i < sizeof(faults)/sizeof(faults[0]); ++i)
    if (state.find(faults[i]) != std::string::npos) return static_cast<uint16_t>(i + 2);
  return 1;
}
struct ServiceResult { bool success; std::string message; };
}

class CartesianSerialBridge {
  using HomeClient = actionlib::SimpleActionClient<aubo_ros_control::MoveHomeAction>;
 public:
  CartesianSerialBridge() : private_("~"), home_("/cartesian_jog/move_home", false) {
    private_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_.param("baud", baud_, 115200);
    private_.param("max_speed_mm_s", max_speed_, 100.0);
    if (!std::isfinite(max_speed_) || max_speed_ <= 0 || max_speed_ > 100)
      throw std::runtime_error("max_speed_mm_s must be in (0, 100]");
    std::string joint_topic;
    private_.param<std::string>("joint_states_topic", joint_topic, "/joint_states");
    nh_.param<std::string>("/aubo_cartesian_jog/base_link", base_, "base_link");
    nh_.param<std::string>("/aubo_cartesian_jog/tool_link", tool_, "tcp_link");
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_jog/command", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/remote/cartesian_status", 1, true);
    state_sub_ = nh_.subscribe<std_msgs::String>("/cartesian_jog/state", 1,
      [this](const std_msgs::String::ConstPtr& msg) {
        state_ = msg->data; state_time_ = abr::monotonicNow(); ++state_revision_;
      });
    joint_sub_ = nh_.subscribe(joint_topic, 1, &CartesianSerialBridge::joints, this);
    enable_ = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    reset_ = nh_.serviceClient<std_srvs::Trigger>("/cartesian_jog/reset");
  }
  ~CartesianSerialBridge() {
    enable_.shutdown(); reset_.shutdown();
    if (service_.valid()) service_.wait();
  }
  void run() {
    double reopen = 0;
    std::unique_ptr<abr::PosixSerial> serial;
    std::unique_ptr<abr::JogPoller> poller;
    abr::SerialDiagnostics diagnostics;
    while (ros::ok() && !stopping) {
      ros::spinOnce();
      tick();
      try {
        if (!serial && abr::monotonicNow() >= reopen) {
          serial.reset(new abr::PosixSerial(port_, baud_));
          poller.reset(new abr::JogPoller(std::random_device{}()));
          diagnostics = abr::SerialDiagnostics{};
          ROS_INFO("Cartesian remote serial %s, 115200 8N1, types 0x11/0x12/0x13", port_.c_str());
        }
        if (serial) {
          abr::Frame poll{};
          if (poller->poll(abr::monotonicNow(), poll)) serial->write(poll);
          for (const auto& frame : serial->read()) {
            RemoteJog_Input input{};
            if (!poller->receive(frame, abr::monotonicNow(), input)) continue;
            diagnostics.accepted();
            receive(input);
            tick();
            abr::Frame reply{};
            RemoteJog_EncodeFeedback(feedback(), frame.Sequence, &reply);
            serial->write(reply);
          }
          diagnostics.report(*serial, poller->online(abr::monotonicNow()),
            "expected=0x12 controller=" + (fresh() ? state_ : "UNKNOWN") +
            " input_valid=" + std::to_string(valid(input_)) +
            " deadman=" + std::to_string(bool(input_.flags & REMOTE_JOG_DEADMAN)) +
            " armed=" + std::to_string(armed_) + " ack=" + std::to_string(ack_.state) + " " + ack_.message);
        } else ros::WallDuration(.005).sleep();
      } catch (const std::exception& error) {
        ROS_WARN_THROTTLE(2., "Cartesian serial: %s", error.what());
        halt(); input_time_ = -1e9;
        serial.reset(); poller.reset(); reopen = abr::monotonicNow() + 1.;
      }
    }
    // 正常SIGINT仍保留ROS连接，发零速度、cancel与禁用请求，等待最多2秒确认停止。
    halt();
    const double until = abr::monotonicNow() + 2.;
    while (ros::ok() && abr::monotonicNow() < until) {
      ros::spinOnce(); tick();
      if (!service_.valid() && !disable_pending_ && !home_running_ && fresh() && state_ == "DISABLED") break;
      ros::WallDuration(.01).sleep();
    }
  }
 private:
  bool fresh() const { return abr::monotonicNow() - state_time_ < .3; }
  bool linked() const { return abr::monotonicNow() - input_time_ < .10 && valid(input_); }
  void reply(const abr::Request& request, uint8_t state, const std::string& message) {
    abr::Reply item{request, state, message}; ledger_.save(item);
    if (request.session == ack_.request.session && request.id == ack_.request.id &&
        request.command == ack_.request.command) ack_ = item;
  }
  void halt() {
    const bool needs_disable = armed_ || home_running_ || enable_confirmation_.pending() || !fresh() || state_ != "DISABLED" ||
      (service_.valid() && job_request_.command == REMOTE_JOG_ENABLE);
    ++generation_; armed_ = false; neutral_seen_ = false;
    if (enable_confirmation_.pending()) {
      enable_confirmation_.cancel();
      reply(enable_request_, REMOTE_FAILED, "使能确认已撤销，请回中松键后重新使能");
    }
    if (home_running_) { home_state_ = 6; home_.cancelGoal(); }
    disable_pending_ = disable_pending_ || needs_disable;
    publishTwist(false);
  }
  void receive(const RemoteJog_Input& input) {
    const bool new_session = !have_session_ || session_ != input.session;
    if (new_session) {
      halt(); have_session_ = true; session_ = input.session; last_id_ = input.request_id;
      last_command_ = input.command; ack_ = abr::Reply{}; ack_.request.session = session_;
      input_ = input; input_time_ = abr::monotonicNow();
      if (input.request_id) {
        ack_.request = abr::Request{session_, input.request_id, input.command};
        reply(ack_.request, REMOTE_REJECTED, "新会话首帧仅建立基线，请重新操作");
      }
      neutral_seen_ = valid(input) && neutral(input);
      return;
    }
    input_ = input; input_time_ = abr::monotonicNow();
    if (!valid(input)) { halt(); return; }
    if (!armed_ && neutral(input)) neutral_seen_ = true;
    if (!input.command) return;
    const abr::Request request{session_, input.request_id, input.command};
    // 相同编号仅回放结果；改变同编号命令或倒退编号均不执行。
    if (input.request_id == last_id_) {
      if (input.command != last_command_) {
        ack_ = abr::Reply{request, REMOTE_REJECTED, "同编号命令冲突"}; halt();
      } else {
        abr::Reply previous;
        if (ledger_.find(request, previous) == abr::Ledger::Duplicate) ack_ = previous;
      }
      return;
    }
    const uint16_t delta = static_cast<uint16_t>(input.request_id - last_id_);
    if (delta >= 32768) { ack_ = abr::Reply{request, REMOTE_REJECTED, "请求编号倒退"}; return; }
    last_id_ = input.request_id; last_command_ = input.command;
    ack_ = abr::Reply{request, REMOTE_ACCEPTED, "收到请求"};
    if (input.command == REMOTE_JOG_STOP || input.command == REMOTE_JOG_CANCEL) {
      stop_request_ = request; have_stop_request_ = true;
      halt(); reply(request, REMOTE_CANCELING, "等待减速停止确认"); return;
    }
    if (!fresh() || !linked() || !neutral_seen_ || !neutral(input)) {
      reply(request, REMOTE_REJECTED, "请回中、松开允许运动键，并等待控制器就绪"); return;
    }
    if (service_.valid() || enable_confirmation_.pending() || disable_pending_ || home_running_ || have_stop_request_) {
      reply(request, REMOTE_REJECTED, "BUSY"); return;
    }
    if (input.command == REMOTE_JOG_ENABLE) {
      if (state_ != "DISABLED") { reply(request, REMOTE_REJECTED, "控制器必须为DISABLED"); return; }
      launchService(request);
    } else if (input.command == REMOTE_JOG_RESET) {
      if (armed_) { reply(request, REMOTE_REJECTED, "先停止并禁用"); return; }
      launchService(request);
    } else if (input.command == REMOTE_JOG_HOME) {
      if (armed_ || state_ != "DISABLED" || !home_.isServerConnected()) {
        reply(request, REMOTE_REJECTED, "回home要求禁用、停稳且action在线"); return;
      }
      home_request_ = request; home_running_ = true; home_state_ = 0; home_error_ = 65535;
      const uint32_t goal_generation = ++home_generation_;
      home_.sendGoal(aubo_ros_control::MoveHomeGoal(),
        [this, request, goal_generation](const actionlib::SimpleClientGoalState& state, const aubo_ros_control::MoveHomeResultConstPtr& result) {
          if (!home_running_ || goal_generation != home_generation_) return;
          home_running_ = false;
          uint8_t code = REMOTE_FAILED;
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result && result->success) code = REMOTE_SUCCEEDED;
          else if (state == actionlib::SimpleClientGoalState::PREEMPTED || state == actionlib::SimpleClientGoalState::RECALLED) code = REMOTE_CANCELED;
          else if (state == actionlib::SimpleClientGoalState::REJECTED) code = REMOTE_REJECTED;
          home_state_ = state == actionlib::SimpleClientGoalState::SUCCEEDED ? 3 :
                        state == actionlib::SimpleClientGoalState::PREEMPTED ? 2 :
                        state == actionlib::SimpleClientGoalState::RECALLED ? 8 :
                        state == actionlib::SimpleClientGoalState::REJECTED ? 5 :
                        state == actionlib::SimpleClientGoalState::LOST ? 9 : 4;
          reply(request, code, result ? result->message : state.toString());
        }, [this, request, goal_generation]() {
          if (!home_running_ || goal_generation != home_generation_) return;
          if (home_state_ != 6) home_state_ = 1;
          reply(request, home_state_ == 6 ? REMOTE_CANCELING : REMOTE_EXECUTING, "回home中，可取消");
        },
        [this, request, goal_generation](const aubo_ros_control::MoveHomeFeedbackConstPtr& value) {
          // feedback与result来自不同话题，终态后迟到的反馈不能把DONE覆盖成STOPPING。
          if (!home_running_ || goal_generation != home_generation_) return;
          home_error_ = static_cast<uint16_t>(std::min(65534., std::max(0., value->max_joint_error * 1000.)));
          if (value->stopping) reply(request, REMOTE_CANCELING, "回home减速中");
        });
      reply(request, REMOTE_ACCEPTED, "已发送home目标");
    }
  }
  void launchService(const abr::Request& request) {
    job_request_ = request; job_generation_ = generation_;
    auto enable_client = enable_, reset_client = reset_;
    service_ = std::async(std::launch::async, [request, enable_client, reset_client]() mutable {
      if (request.command == REMOTE_JOG_RESET) {
        std_srvs::Trigger call;
        const bool ok = reset_client.exists() && reset_client.call(call);
        return ServiceResult{ok && call.response.success, ok ? call.response.message : "reset服务不可用"};
      }
      std_srvs::SetBool call; call.request.data = request.command == REMOTE_JOG_ENABLE;
      const bool ok = enable_client.exists() && enable_client.call(call);
      return ServiceResult{ok && call.response.success, ok ? call.response.message : "使能服务不可用"};
    });
  }
  void tick() {
    const double now = abr::monotonicNow();
    const bool enabling = enable_confirmation_.pending() ||
      (service_.valid() && job_request_.command == REMOTE_JOG_ENABLE && job_generation_ == generation_);
    if ((!linked() || !fresh() || (enabling && !neutral(input_))) &&
        (armed_ || home_running_ || enabling)) halt();
    if (fresh() && state_.find("FAULT:") == 0 && (armed_ || enabling)) halt();
    if (service_.valid() && service_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto result = service_.get();
      if (job_request_.command == REMOTE_JOG_ENABLE) {
        const bool accept = result.success && job_generation_ == generation_ && linked() && fresh() &&
          neutral(input_) && !disable_pending_ && state_.find("FAULT:") != 0;
        armed_ = false;
        if (accept) {
          enable_request_ = job_request_;
          enable_confirmation_.begin(state_revision_, now);
          reply(enable_request_, REMOTE_ACCEPTED, "使能服务成功，等待控制器ENABLED_IDLE确认");
        } else {
          reply(job_request_, REMOTE_FAILED, "使能失败或输入条件已变化");
          disable_pending_ = true;
        }
      } else if (job_request_.id) reply(job_request_, result.success ? REMOTE_SUCCEEDED : REMOTE_FAILED, result.message);
      if (job_request_.command == REMOTE_JOG_STOP && !result.success) {
        disable_pending_ = true; next_disable_ = now + .25;
      }
    }
    if (enable_confirmation_.pending()) {
      const auto result = enable_confirmation_.update(now, state_revision_, state_ == "ENABLED_IDLE",
        linked() && fresh() && neutral(input_) && !disable_pending_ && state_.find("FAULT:") != 0);
      if (result == abr::JogEnableConfirmation::Confirmed) {
        armed_ = true;
        reply(enable_request_, REMOTE_SUCCEEDED, "控制器已确认使能，可以按住SW点动");
      } else if (result == abr::JogEnableConfirmation::Failed) {
        reply(enable_request_, REMOTE_FAILED, "使能状态确认超时或输入条件已变化");
        disable_pending_ = true;
        halt();
      }
    }
    if (disable_pending_ && !service_.valid() && now >= next_disable_) {
      disable_pending_ = false;
      launchService(abr::Request{session_, 0, REMOTE_JOG_STOP});
    }
    if (have_stop_request_ && !disable_pending_ && !service_.valid() && !home_running_ && fresh() && state_ == "DISABLED") {
      reply(stop_request_, stop_request_.command == REMOTE_JOG_CANCEL ? REMOTE_CANCELED : REMOTE_SUCCEEDED,
            "已停止，保持禁用"); have_stop_request_ = false;
    }
    if (now - publish_time_ >= .02) {
      publish_time_ = now;
      publishTwist(armed_ && linked() && fresh() && (input_.flags & REMOTE_JOG_DEADMAN) &&
                   (state_ == "ENABLED_IDLE" || state_ == "JOGGING"));
    }
    if (now - status_time_ >= .1) {
      status_time_ = now; std_msgs::String status;
      status.data = "session=" + std::to_string(session_) + " armed=" + std::to_string(armed_) +
        " controller=" + (fresh() ? state_ : "UNKNOWN") + " request=" + std::to_string(ack_.request.id) +
        " ack=" + std::to_string(ack_.state) + " " + ack_.message;
      status_pub_.publish(status);
    }
  }
  void publishTwist(bool moving) {
    geometry_msgs::TwistStamped msg; msg.header.stamp = ros::Time::now();
    msg.header.frame_id = input_.frame ? tool_ : base_;
    if (moving) {
      double norm = 0;
      for (auto axis : input_.axis) norm += static_cast<double>(axis) * axis;
      const double factor = std::min(input_.speed / 10., max_speed_) / 1000. / std::max(1000., std::sqrt(norm));
      msg.twist.linear.x = input_.axis[0] * factor;
      msg.twist.linear.y = input_.axis[1] * factor;
      msg.twist.linear.z = input_.axis[2] * factor;
    }
    twist_pub_.publish(msg);
  }
  void joints(const sensor_msgs::JointState::ConstPtr& msg) {
    joint_mask_ = 0;
    if (msg->name.size() != msg->position.size() ||
        std::set<std::string>(msg->name.begin(), msg->name.end()).size() != msg->name.size()) return;
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (msg->header.stamp.isZero() || age < -.05 || age > .5) return;
    for (std::size_t i = 0; i < 6; ++i) {
      auto found = std::find(msg->name.begin(), msg->name.end(), abr::jointNames()[i]);
      if (found == msg->name.end()) continue;
      const double value = msg->position[found - msg->name.begin()] * 1000.;
      if (!std::isfinite(value) || value < -32768 || value > 32767) continue;
      joints_[i] = static_cast<int16_t>(std::lround(value)); joint_mask_ |= 1 << i;
    }
    joint_time_ = abr::monotonicNow();
  }
  RemoteJog_Feedback* feedback() {
    output_ = RemoteJog_Feedback{}; output_.session = session_;
    output_.request_id = ack_.request.id; output_.command = ack_.request.command; output_.ack_state = ack_.state;
    output_.controller_state = fresh() ? controllerCode(state_) : REMOTE_JOG_UNKNOWN;
    output_.home_state = home_state_; output_.home_error = home_error_;
    output_.flags = (linked() ? 1 : 0) | (armed_ ? 2 : 0) | (fresh() ? 4 : 0) |
      ((service_.valid() || enable_confirmation_.pending() || disable_pending_ || have_stop_request_) ? 8 : 0);
    output_.joint_mask = abr::monotonicNow() - joint_time_ <= .5 ? joint_mask_ : 0;
    for (std::size_t i = 0; i < 6; ++i) output_.joints[i] = (output_.joint_mask & (1 << i)) ? joints_[i] : 0;
    output_.fault = fresh() ? faultCode(state_) : 1;
    return &output_;
  }
  ros::NodeHandle nh_, private_;
  HomeClient home_;
  ros::Publisher twist_pub_, status_pub_;
  ros::Subscriber state_sub_, joint_sub_;
  ros::ServiceClient enable_, reset_;
  std::future<ServiceResult> service_;
  abr::Request job_request_, home_request_, stop_request_, enable_request_;
  abr::JogEnableConfirmation enable_confirmation_;
  abr::Ledger ledger_; abr::Reply ack_;
  RemoteJog_Input input_{}; RemoteJog_Feedback output_{};
  std::string port_, base_, tool_, state_;
  int baud_ = 115200; double max_speed_ = 100;
  uint32_t session_ = 0, generation_ = 0, job_generation_ = 0;
  uint32_t home_generation_ = 0;
  uint64_t state_revision_ = 0;
  uint16_t last_id_ = 0, home_error_ = 65535;
  uint8_t last_command_ = 0, home_state_ = 255, joint_mask_ = 0;
  std::array<int16_t, 6> joints_{};
  bool have_session_ = false, armed_ = false, neutral_seen_ = false, disable_pending_ = false;
  bool home_running_ = false, have_stop_request_ = false;
  double input_time_ = -1e9, state_time_ = -1e9, joint_time_ = -1e9;
  double next_disable_ = 0, publish_time_ = 0, status_time_ = 0;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "cartesian_serial_bridge", ros::init_options::NoSigintHandler);
  // 仅设置字符编码；保留ROS数值解析使用的C locale。
  std::setlocale(LC_CTYPE, "");
  std::signal(SIGINT, stopSignal); std::signal(SIGTERM, stopSignal);
  try { CartesianSerialBridge node; node.run(); }
  catch (const std::exception& error) { ROS_FATAL("Cartesian serial bridge: %s", error.what()); return 1; }
  return 0;
}
