#include "aubo_remote_bridge/jog_remote_core.hpp"
namespace aubo_remote_bridge {
bool JogPoller::poll(double now, Frame& frame) {
  if (pending_ && now - sent_ < .10) return false;
  if (now < next_) return false;
  if (rotate_ || pending_ || sequence_ == 65535) {
    ++session_; sequence_ = 0; received_ = -1;
    had_valid_ = false; rotate_ = false;
  }
  ++sequence_;
  frame = Frame{}; frame.Type = REMOTE_JOG_POLL; frame.Sequence = sequence_; frame.Length = 4;
  Remote_PutU32(frame.Payload, session_);
  sent_ = now; next_ = now + .02; pending_ = true; return true;
}
bool JogPoller::receive(const Frame& frame, double now, RemoteJog_Input& input) {
  if (!pending_ || frame.Sequence != sequence_ || now < sent_ || now - sent_ >= .10 ||
      !RemoteJog_DecodeInput(&frame, &input) || input.session != session_) return false;
  pending_ = false; received_ = now;
  if ((input.flags & 3) == 3) had_valid_ = true;
  else if (had_valid_) {
    // 快速复位/重新校准时仍可能及时回复POLL，不能仅依赖串口超时重建请求基线。
    rotate_ = true; had_valid_ = false;
  }
  return true;
}
JogEnableConfirmation::Result JogEnableConfirmation::update(
    double now, uint64_t revision, bool idle, bool permitted) {
  if (!pending_) return Waiting;
  if (!permitted || now - started_ >= 1.0) {
    pending_ = false; return Failed;
  }
  if (revision > revision_ && idle) {
    pending_ = false; return Confirmed;
  }
  return Waiting;
}
}  // namespace aubo_remote_bridge
