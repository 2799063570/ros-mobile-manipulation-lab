/**
 * 功能：将手持端的INPUT帧发布为ROS输入/离散请求，再把ROS关节和任务状态封装回串口。
 * 分工：本进程只收发与映射，不规划、不连接AUBO SDK；耗时动作由独立命名位置节点处理。
 * 线程：主循环调用spinOnce，订阅回调与串口处理串行执行，无共享数据竞争。
 * 时钟：报文截止使用steady_clock；ROS时间只用于消息时间戳和关节消息时间检查。
 */
#include "aubo_remote_bridge/posix_serial.hpp"
#include <aubo_remote_bridge/CommandStatus.h>
#include <aubo_remote_bridge/RemoteCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <cmath>
#include <limits>
#include <random>
#include <set>

namespace abr = aubo_remote_bridge;
class SerialBridge {
 public:
  SerialBridge() : private_("~") {
    private_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_.param("baud", baud_, 115200);
    private_.param("enable_commands", commands_, false);
    std::string topic;
    private_.param<std::string>("joint_states_topic", topic, "/aubo_i5/joint_states");
    joy_ = nh_.advertise<sensor_msgs::Joy>("/remote/joy", 1);
    connected_ = nh_.advertise<std_msgs::Bool>("/remote/connected", 1, true);
    valid_ = nh_.advertise<std_msgs::Bool>("/remote/input_valid", 1, true);
    status_ = nh_.advertise<std_msgs::String>("/remote/status", 1, true);
    command_ = nh_.advertise<abr::RemoteCommand>("/remote/command", 1);
    joint_sub_ = nh_.subscribe(topic, 1, &SerialBridge::joints, this);
    result_sub_ = nh_.subscribe("/remote/command_result", 10, &SerialBridge::result, this);
    motion_sub_ = nh_.subscribe("/remote/motion_state", 1, &SerialBridge::motion, this);
  }
  void run() {
    // 串口轮询与ROS回调都在本线程；运动在另一个节点，耗时规划不会阻塞此链路。
    while (ros::ok()) {
      publishNeutral(); setConnected(false); text("DISCONNECTED");
      try {
        abr::PosixSerial serial(port_, baud_);
        abr::Poller core(std::random_device{}());
        ack_ = abr::Reply{}; last_request_ = abr::Request{};
        bool was_online = false;
        text("WAITING_INPUT_V2");
        ROS_INFO("串口C++桥已打开%s，115200 8N1；命令转发=%s", port_.c_str(), commands_ ? "开启" : "关闭");
        while (ros::ok()) {
          ros::spinOnce();
          abr::Frame poll{};
          if (core.request(abr::monotonicNow(), poll)) serial.write(poll);
          for (const auto& frame : serial.read()) {
            abr::Input input;
            const double now = abr::monotonicNow();
            if (!core.receive(frame, now, input)) continue;
            publishInput(input);
            abr::Request request;
            if (core.command(input, request)) {
              last_request_ = request;
              ack_ = abr::Reply{request, REMOTE_UNKNOWN, "等待命令节点确认"};
              if (commands_ && command_.getNumSubscribers() > 0) {
                abr::RemoteCommand message; message.session = request.session;
                message.request_id = request.id; message.command = request.command;
                command_.publish(message);
              } else ack_ = abr::Reply{request, REMOTE_REJECTED, "命令转发关闭或请求节点未连接"};
            } else if (input.valid() && input.request_id &&
                       (last_request_.session != core.session() || last_request_.id != input.request_id)) {
              // 重连首帧只建立基线；把被忽略的旧请求明确回报为REJECT，避免屏幕永久显示WAIT。
              last_request_ = abr::Request{core.session(), input.request_id, input.command};
              ack_ = abr::Reply{last_request_, REMOTE_REJECTED, "恢复连接时忽略旧请求，请重新确认"};
            }
            // 关节消息过期时清有效位；绝不把缓存或默认零值包装为实时关节角。
            const uint8_t mask = now - joint_time_ <= .5 ? mask_ : 0;
            const uint8_t state = now - motion_time_ <= .5 ? motion_state_ : REMOTE_UNKNOWN;
            serial.write(abr::feedback(core.session(), frame.Sequence, mask ? REMOTE_SOURCE_ROS : REMOTE_SOURCE_NONE,
                                      mask, joints_, ack_, state, motion_target_));
          }
          const bool online = core.online(abr::monotonicNow());
          if (was_online != online) {
            was_online = online; setConnected(online);
            if (!online) { publishNeutral(); text("INPUT_TIMEOUT"); }
          }
        }
      } catch (const std::exception& error) { ROS_WARN_THROTTLE(2., "串口通信异常：%s", error.what()); }
      publishNeutral(); setConnected(false); text("DISCONNECTED");
      const double retry = abr::monotonicNow() + 1.;
      while (ros::ok() && abr::monotonicNow() < retry) { ros::spinOnce(); ros::WallDuration(.02).sleep(); }
    }
  }
 private:
  void publishInput(const abr::Input& input) {
    sensor_msgs::Joy message; message.header.stamp = ros::Time::now(); message.header.frame_id = "handheld_remote";
    message.axes = {input.valid() ? input.x / 1000.f : 0.f, input.valid() ? input.y / 1000.f : 0.f};
    message.buttons = {input.valid() ? input.button : 0}; joy_.publish(message);
    std_msgs::Bool valid; valid.data = input.valid(); valid_.publish(valid);
    text(input.valid() ? "INPUT_OK" : "CALIBRATING_OR_ADC_ERROR");
  }
  void publishNeutral() { publishInput(abr::Input{}); }
  void setConnected(bool value) { std_msgs::Bool message; message.data = value; connected_.publish(message); }
  void text(const std::string& value) { std_msgs::String message; message.data = value; status_.publish(message); }
  void joints(const sensor_msgs::JointState::ConstPtr& message) {
    // 回调只维护最近一份反馈；每帧重新置有效位，缺失关节不从上一帧补齐。
    mask_ = 0; joints_.fill(0); joint_time_ = abr::monotonicNow();
    const double age = (ros::Time::now() - message->header.stamp).toSec();
    if (!message->header.stamp.isZero() && (age < 0 || age > .5)) return;
    if (message->name.size() != message->position.size() || std::set<std::string>(message->name.begin(), message->name.end()).size() != message->name.size()) return;
    for (std::size_t i = 0; i < abr::jointNames().size(); ++i) {
      for (std::size_t j = 0; j < message->name.size(); ++j) if (message->name[j] == abr::jointNames()[i]) {
        const double value = message->position[j] * 1000.;
        if (std::isfinite(value) && value >= -32768 && value <= 32767) {
          joints_[i] = static_cast<int16_t>(std::lround(value)); mask_ |= static_cast<uint8_t>(1u << i);
        }
      }
    }
  }
  void result(const abr::CommandStatus::ConstPtr& message) {
    // 逐请求结果只接受当前会话/编号，旧任务的迟到结果不能覆盖新请求的应答。
    if (message->session == last_request_.session && message->request_id == last_request_.id && message->command == last_request_.command && message->state <= REMOTE_UNKNOWN)
      ack_ = abr::Reply{last_request_, message->state, message->message};
  }
  void motion(const abr::CommandStatus::ConstPtr& message) {
    // 当前动作独立于最新请求：例如第二请求BUSY时，仍显示第一动作RUN HOME。
    if (message->state > REMOTE_UNKNOWN || message->command > REMOTE_CANCEL) return;
    motion_time_ = abr::monotonicNow(); motion_state_ = message->state; motion_target_ = message->command;
  }
  ros::NodeHandle nh_, private_;
  ros::Publisher joy_, connected_, valid_, status_, command_;
  ros::Subscriber joint_sub_, result_sub_, motion_sub_;
  std::string port_; int baud_ = 115200; bool commands_ = false;
  std::array<int16_t, 6> joints_{}; uint8_t mask_ = 0;
  double joint_time_ = -1e9, motion_time_ = -1e9;
  uint8_t motion_state_ = REMOTE_UNKNOWN, motion_target_ = 0;
  abr::Request last_request_; abr::Reply ack_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "aubo_remote_bridge");
  try { SerialBridge bridge; bridge.run(); }
  catch (const std::exception& error) { ROS_FATAL("串口节点启动失败：%s", error.what()); return 1; }
  return 0;
}
