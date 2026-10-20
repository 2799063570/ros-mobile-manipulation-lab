#pragma once
#include "posix_serial.hpp"
#include <ros/ros.h>

namespace aubo_remote_bridge {
// 每次打开串口重新统计；用墙上时间，Gazebo暂停时也能输出诊断。
class SerialDiagnostics {
 public:
  void accepted() { ++accepted_; }
  void report(const PosixSerial& serial, bool online, const std::string& state) {
    const double now = monotonicNow();
    if (!started_) { started_ = true; next_ = now + 2.; }
    if (now < next_) return;
    next_ = now + 2.;
    const char* hint = online ? "INPUT_OK" :
      serial.receivedBytes() == bytes_ ? "NO_RX: check port/TX/RX/GND/firmware poll response" :
      serial.receivedFrames() == frames_ ? "NO_FRAME: check 115200 8N1, A5 5A, v2, length/CRC" :
      "INPUT_REJECTED: check protocol type, session, sequence, fields and 100ms deadline";
    ROS_INFO_STREAM("[serial_rx] bytes=" << serial.receivedBytes()
      << " frames=" << serial.receivedFrames() << " errors=" << serial.parserErrors()
      << " accepted=" << accepted_ << " last_type=0x" << std::hex << serial.lastType()
      << std::dec << " " << hint << " " << state);
    bytes_ = serial.receivedBytes(); frames_ = serial.receivedFrames();
  }
 private:
  bool started_ = false;
  double next_ = 0;
  uint64_t accepted_ = 0, bytes_ = 0, frames_ = 0;
};
}  // namespace aubo_remote_bridge
