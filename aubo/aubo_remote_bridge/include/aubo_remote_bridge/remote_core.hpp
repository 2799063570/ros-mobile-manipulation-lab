#pragma once
// 本核心不依赖ROS，可在电脑端直接编译，验证真实串口协议和请求去重。
extern "C" {
#include "aubo_remote_bridge/RemoteProtocol.h"
}
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace aubo_remote_bridge {
using Frame = Remote_FrameTypeDef;
struct Input {
  uint16_t raw_x = 0, raw_y = 0, request_id = 0;
  int16_t x = 0, y = 0;
  uint8_t button = 0, flags = 0, page = 0, command = 0;
  bool valid() const { return flags == 3; }
};
struct Request { uint32_t session = 0; uint16_t id = 0; uint8_t command = 0; };
struct Reply { Request request; uint8_t state = REMOTE_IDLE; std::string message; };

class Poller {
 public:
  explicit Poller(uint32_t session, double period = .02, double timeout = .1);
  bool request(double now, Frame& output);
  bool receive(const Frame& frame, double now, Input& input);
  bool command(const Input& input, Request& request);
  bool online(double now) const;
  uint32_t session() const { return session_; }
 private:
  uint32_t session_;
  uint16_t sequence_ = 0, last_request_ = 0;
  double period_, timeout_, sent_ = 0, next_ = 0, received_ = -1;
  bool pending_ = false, baseline_ = false, had_valid_ = false, rotate_ = false;
};

// 有界缓存让ROS服务重试和串口重复帧共用幂等语义；冲突编号不得偷偷替换命令。
class Ledger {
 public:
  enum Lookup { Missing, Duplicate, Conflict };
  Lookup find(const Request& request, Reply& result) const;
  void save(const Reply& reply);
 private:
  std::deque<Reply> replies_;
};

std::vector<uint8_t> encode(const Frame& frame);
Frame feedback(uint32_t session, uint16_t sequence, uint8_t source, uint8_t mask,
               const std::array<int16_t, 6>& joints, const Reply& ack,
               uint8_t motion_state, uint8_t target);
const std::array<std::string, 6>& jointNames();
}  // namespace aubo_remote_bridge
