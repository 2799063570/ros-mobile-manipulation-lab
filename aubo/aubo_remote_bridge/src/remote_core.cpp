#include "aubo_remote_bridge/remote_core.hpp"
#include <stdexcept>

namespace aubo_remote_bridge {
Poller::Poller(uint32_t session, double period, double timeout)
    : session_(session), period_(period), timeout_(timeout) {
  if (!(period > 0 && period <= timeout && timeout <= .5)) throw std::invalid_argument("轮询参数无效");
}
bool Poller::request(double now, Frame& output) {
  if (received_ >= 0 && now - received_ >= timeout_) baseline_ = false;
  if (pending_) {
    if (now - sent_ < timeout_) return false;
    pending_ = false; received_ = -1; baseline_ = false; rotate_ = true;
  }
  if (now < next_) return false;
  // 超时或ADC重新校准可能意味着MCU复位；换会话避免其请求编号从1重新计数后撞上旧缓存。
  if (rotate_) { ++session_; sequence_ = 0; rotate_ = false; baseline_ = false; had_valid_ = false; }
  if (++sequence_ == 0) { ++session_; sequence_ = 1; baseline_ = false; received_ = -1; }
  output = Frame{}; output.Type = REMOTE_POLL; output.Sequence = sequence_; output.Length = 4;
  Remote_PutU32(output.Payload, session_);
  sent_ = now; next_ = now + period_; pending_ = true;
  return true;
}
bool Poller::receive(const Frame& frame, double now, Input& input) {
  if (!pending_ || frame.Type != REMOTE_INPUT || frame.Length != 18 || frame.Sequence != sequence_ ||
      now < sent_ || now - sent_ >= timeout_ || Remote_GetU32(frame.Payload) != session_) return false;
  Input data;
  data.raw_x = Remote_GetU16(frame.Payload + 4); data.raw_y = Remote_GetU16(frame.Payload + 6);
  data.x = Remote_GetI16(frame.Payload + 8); data.y = Remote_GetI16(frame.Payload + 10);
  data.button = frame.Payload[12]; data.flags = frame.Payload[13]; data.page = frame.Payload[14];
  data.command = frame.Payload[15]; data.request_id = Remote_GetU16(frame.Payload + 16);
  if (data.raw_x > 4095 || data.raw_y > 4095 || data.x < -1000 || data.x > 1000 || data.y < -1000 || data.y > 1000 ||
      data.button > 1 || data.flags > 3 || data.page > 4 || data.command > REMOTE_CANCEL ||
      ((data.command == 0) != (data.request_id == 0)) ||
      (!data.valid() && (data.x || data.y || data.button))) return false;
  pending_ = false; received_ = now; input = data;
  if (!data.valid()) { baseline_ = false; if (had_valid_) rotate_ = true; }
  else had_valid_ = true;
  return true;
}
bool Poller::command(const Input& input, Request& result) {
  if (!input.valid()) { baseline_ = false; return false; }
  // 新会话或超时恢复后，第一帧仅记录基线，避免电脑重启后执行手持端缓存的旧请求。
  if (!baseline_) { last_request_ = input.request_id; baseline_ = true; return false; }
  if (last_request_ == input.request_id) return false;
  last_request_ = input.request_id;
  if (!input.request_id || !input.command) return false;
  result = Request{session_, input.request_id, input.command}; return true;
}
bool Poller::online(double now) const { return received_ >= 0 && now >= received_ && now - received_ < timeout_; }

Ledger::Lookup Ledger::find(const Request& request, Reply& result) const {
  for (const auto& reply : replies_) if (reply.request.session == request.session && reply.request.id == request.id) {
    result = reply;
    return reply.request.command == request.command ? Duplicate : Conflict;
  }
  return Missing;
}
void Ledger::save(const Reply& reply) {
  for (auto& item : replies_) if (item.request.session == reply.request.session && item.request.id == reply.request.id) {
    item = reply; return;
  }
  if (replies_.size() == 64) replies_.pop_front();
  replies_.push_back(reply);
}
std::vector<uint8_t> encode(const Frame& frame) {
  std::vector<uint8_t> output(REMOTE_MAX_FRAME);
  const auto size = Remote_Encode(&frame, output.data());
  if (!size) throw std::invalid_argument("载荷过长");
  output.resize(size); return output;
}
Frame feedback(uint32_t session, uint16_t sequence, uint8_t source, uint8_t mask,
               const std::array<int16_t, 6>& joints, const Reply& ack, uint8_t motion_state, uint8_t target) {
  Frame output{}; output.Type = REMOTE_FEEDBACK; output.Sequence = sequence; output.Length = 24;
  Remote_PutU32(output.Payload, session); output.Payload[4] = source; output.Payload[5] = mask;
  for (std::size_t i = 0; i < 6; ++i) Remote_PutU16(output.Payload + 6 + i * 2, static_cast<uint16_t>(joints[i]));
  Remote_PutU16(output.Payload + 18, ack.request.id); output.Payload[20] = ack.request.command;
  output.Payload[21] = ack.state; output.Payload[22] = motion_state; output.Payload[23] = target;
  return output;
}
const std::array<std::string, 6>& jointNames() {
  static const std::array<std::string, 6> names{{"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"}};
  return names;
}
}  // namespace aubo_remote_bridge
