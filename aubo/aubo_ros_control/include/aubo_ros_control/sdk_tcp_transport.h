#ifndef AUBO_ROS_CONTROL_SDK_TCP_TRANSPORT_H
#define AUBO_ROS_CONTROL_SDK_TCP_TRANSPORT_H

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// The bundled SDK exposes no socket accessor. Match only this process's IPv4
// TCP connections to the configured controller endpoint; never scan other PIDs,
// change system TCP settings, or rely on the SDK's private object layout.
class SdkTcpTransport
{
public:
  bool configure(const std::string& host, int port)
  {
    sockets_.clear();
    if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &address_) != 1)
      return false;
    port_ = htons(static_cast<unsigned short>(port));
    DIR* directory = opendir("/proc/self/fd");
    if (!directory)
      return false;
    bool success = true;
    while (const auto* entry = readdir(directory))
    {
      char* end = nullptr;
      const long value = std::strtol(entry->d_name, &end, 10);
      if (end == entry->d_name || *end != '\0' || value < 0 ||
          value > std::numeric_limits<int>::max())
        continue;
      const int fd = static_cast<int>(value);
      if (!matches(fd))
        continue;
      const int enabled = 1;
      if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0 ||
          setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &enabled, sizeof(enabled)) != 0)
        success = false;
      else
        sockets_.push_back(fd);
    }
    closedir(directory);
    return success && !sockets_.empty();
  }

  std::size_t size() const { return sockets_.size(); }

  // QUICKACK is temporary on Linux: re-arm before synchronous SDK requests.
  // Descriptors are borrowed, never closed here. Revalidate each endpoint so a
  // closed/reused descriptor cannot cause us to configure an unrelated socket.
  bool prepare() const
  {
    if (sockets_.empty())
      return false;
    const int enabled = 1;
    for (const int fd : sockets_)
      if (!matches(fd) ||
          setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &enabled, sizeof(enabled)) != 0)
        return false;
    return true;
  }

private:
  bool matches(int fd) const
  {
    sockaddr_in peer{};
    socklen_t length = sizeof(peer);
    int type = 0;
    socklen_t type_length = sizeof(type);
    return getpeername(fd, reinterpret_cast<sockaddr*>(&peer), &length) == 0 &&
           length == sizeof(peer) && peer.sin_family == AF_INET &&
           peer.sin_addr.s_addr == address_.s_addr && peer.sin_port == port_ &&
           getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &type_length) == 0 &&
           type == SOCK_STREAM;
  }

  in_addr address_{};
  in_port_t port_{};
  std::vector<int> sockets_;
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif
