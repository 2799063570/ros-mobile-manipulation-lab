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
  bool pending_ = false;
};
}  // namespace aubo_remote_bridge
