#pragma once
#include "remote_core.hpp"
extern "C" {
#include "RemoteJogProtocol.h"
}
namespace aubo_remote_bridge {
// 独立报文类型；不会将旧HOME/ZERO遥控器的输入解释为点动。
class JogPoller {
 public:
  explicit JogPoller(uint32_t session) : session_(session) {}
  bool poll(double now, Frame& frame);
  bool receive(const Frame& frame, double now, RemoteJog_Input& input);
  bool online(double now) const { return received_ >= 0 && now - received_ < .10; }
  uint32_t session() const { return session_; }
 private:
  uint32_t session_;
  uint16_t sequence_ = 0;
  double sent_ = 0, next_ = 0, received_ = -1;
  bool pending_ = false, had_valid_ = false, rotate_ = false;
};

// 服务成功不等于控制器状态已同步。等待服务返回之后的新状态再完成使能。
class JogEnableConfirmation {
 public:
  enum Result { Waiting, Confirmed, Failed };
  void begin(uint64_t revision, double now) {
    revision_ = revision; started_ = now; pending_ = true;
  }
  void cancel() { pending_ = false; }
  bool pending() const { return pending_; }
  Result update(double now, uint64_t revision, bool idle, bool permitted);
 private:
  uint64_t revision_ = 0;
  double started_ = 0;
  bool pending_ = false;
};
}  // namespace aubo_remote_bridge
