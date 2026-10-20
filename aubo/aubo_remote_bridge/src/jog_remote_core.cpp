#include "aubo_remote_bridge/jog_remote_core.hpp"
namespace aubo_remote_bridge {
bool JogPoller::poll(double now, Frame& frame) {
  if (pending_ && now - sent_ < .10) return false;
  if (now < next_) return false;
  if (pending_ || sequence_ == 65535) { ++session_; sequence_ = 0; received_ = -1; }
  ++sequence_;
  frame = Frame{}; frame.Type = REMOTE_JOG_POLL; frame.Sequence = sequence_; frame.Length = 4;
  Remote_PutU32(frame.Payload, session_);
  sent_ = now; next_ = now + .02; pending_ = true; return true;
}
bool JogPoller::receive(const Frame& frame, double now, RemoteJog_Input& input) {
  if (!pending_ || frame.Sequence != sequence_ || now < sent_ || now - sent_ >= .10 ||
      !RemoteJog_DecodeInput(&frame, &input) || input.session != session_) return false;
  pending_ = false; received_ = now; return true;
}
}  // namespace aubo_remote_bridge
