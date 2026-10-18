#pragma once
// ROS Noetic运行在Linux：直接使用termios，不额外依赖Python或第三方串口库。
#include "aubo_remote_bridge/remote_core.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace aubo_remote_bridge {
inline double monotonicNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class PosixSerial {
 public:
  explicit PosixSerial(const std::string& path, int baud) {
    if (baud != 115200) throw std::invalid_argument("当前串口固件固定为115200，请保持两端一致");
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) fail("打开串口");
    try {
      if (::ioctl(fd_, TIOCEXCL) < 0) fail("串口独占");
      termios settings{};
      if (::tcgetattr(fd_, &settings) < 0) fail("读取串口设置");
      ::cfmakeraw(&settings);
      ::cfsetispeed(&settings, B115200); ::cfsetospeed(&settings, B115200);
      settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
      settings.c_cflag |= CS8 | CLOCAL | CREAD;
      settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
      if (::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIOFLUSH) < 0) fail("设置串口");
    } catch (...) { ::close(fd_); fd_ = -1; throw; }
    Remote_ParserInit(&parser_);
  }
  ~PosixSerial() { if (fd_ >= 0) ::close(fd_); }
  PosixSerial(const PosixSerial&) = delete;
  PosixSerial& operator=(const PosixSerial&) = delete;
  std::vector<Frame> read() {
    pollfd descriptor{fd_, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 10);  // 至多等待10ms，ROS回调不被永久阻塞
    if (ready < 0) { if (errno == EINTR) return {}; fail("等待串口"); }
    if (!ready) return {};
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("串口已断开");
    uint8_t buffer[256];
    const auto size = ::read(fd_, buffer, sizeof(buffer));
    if (size < 0) { if (errno == EAGAIN || errno == EINTR) return {}; fail("读取串口"); }
    std::vector<Frame> output;
    for (ssize_t i = 0; i < size; ++i) {
      Frame frame{};
      if (Remote_ParseByte(&parser_, buffer[i], &frame)) output.push_back(frame);
    }
    return output;
  }
  void write(const Frame& frame) {
    const auto data = encode(frame);
    std::size_t sent = 0;
    const double deadline = monotonicNow() + .05;
    while (sent < data.size()) {
      if (monotonicNow() >= deadline) throw std::runtime_error("串口写入超过50ms，重新建立会话");
      pollfd descriptor{fd_, POLLOUT, 0};
      if (::poll(&descriptor, 1, 5) < 0) { if (errno == EINTR) continue; fail("等待串口写入"); }
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("串口写入时断开");
      const auto size = ::write(fd_, data.data() + sent, data.size() - sent);
      if (size < 0) { if (errno == EAGAIN || errno == EINTR) continue; fail("写入串口"); }
      sent += static_cast<std::size_t>(size);
    }
  }
 private:
  [[noreturn]] static void fail(const char* operation) { throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno)); }
  int fd_ = -1;
  Remote_ParserTypeDef parser_{};
};
}  // namespace aubo_remote_bridge
