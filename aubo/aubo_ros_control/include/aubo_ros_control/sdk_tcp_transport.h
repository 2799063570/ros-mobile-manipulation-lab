#ifndef AUBO_ROS_CONTROL_SDK_TCP_TRANSPORT_H
#define AUBO_ROS_CONTROL_SDK_TCP_TRANSPORT_H

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <array>
#include <cstddef>
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
  struct SocketSample
  {
    int fd{-1};
    std::uint16_t local_port{0};
    bool byte_counters_valid{false};
    std::uint64_t bytes_sent{0}, bytes_acked{0}, bytes_received{0};
    std::uint32_t retransmissions{0}, rtt_us{0}, rto_us{0};
    std::uint32_t last_data_sent_ms{0}, last_data_received_ms{0}, unacked{0}, notsent_bytes{0};
  };
  struct Snapshot
  {
    bool valid{false};
    std::size_t connections{0};
    std::uint64_t total_retransmissions{0}, unacknowledged_segments{0};
    std::uint32_t max_rtt_us{0}, max_rto_us{0};
    std::array<SocketSample, 8> sockets{};
    std::size_t detail_count{0};
  };

  // Per-process controller sockets only. This excludes browser/Wi-Fi traffic
  // from the machine-wide cumulative retransmission counters.
  Snapshot snapshot() const
  {
    Snapshot result;
    if (sockets_.empty()) return result;
    for (int fd : sockets_)
    {
      // Linux TCP_INFO extends the stable glibc tcp_info prefix. Keep the
      // UAPI layout explicit without mixing linux/tcp.h with netinet/tcp.h.
      struct ExtendedInfo
      {
        tcp_info basic;
        std::uint64_t pacing_rate, max_pacing_rate, bytes_acked, bytes_received;
        std::uint32_t segs_out, segs_in, notsent_bytes, min_rtt, data_segs_in, data_segs_out;
        std::uint64_t delivery_rate, busy_time, rwnd_limited, sndbuf_limited;
        std::uint32_t delivered, delivered_ce;
        std::uint64_t bytes_sent;
      } extended{};
      static_assert(offsetof(ExtendedInfo, bytes_acked) == 120, "Unexpected Linux TCP_INFO prefix");
      static_assert(offsetof(ExtendedInfo, bytes_sent) == 200, "Unexpected Linux TCP_INFO extension");
      socklen_t length = sizeof(extended);
      if (!matches(fd) || getsockopt(fd, IPPROTO_TCP, TCP_INFO, &extended, &length) != 0)
        return {};
      const auto& info = extended.basic;
      ++result.connections;
      result.total_retransmissions += info.tcpi_total_retrans;
      result.unacknowledged_segments += info.tcpi_unacked;
      result.max_rtt_us = std::max(result.max_rtt_us, info.tcpi_rtt);
      result.max_rto_us = std::max(result.max_rto_us, info.tcpi_rto);
      if (result.detail_count < result.sockets.size())
      {
        auto& socket = result.sockets[result.detail_count++];
        socket.fd = fd;
        sockaddr_in local{};
        socklen_t local_length = sizeof(local);
        if (getsockname(fd, reinterpret_cast<sockaddr*>(&local), &local_length) != 0) return {};
        socket.local_port = ntohs(local.sin_port);
        socket.retransmissions = info.tcpi_total_retrans;
        socket.rtt_us = info.tcpi_rtt; socket.rto_us = info.tcpi_rto;
        socket.last_data_sent_ms = info.tcpi_last_data_sent;
        socket.last_data_received_ms = info.tcpi_last_data_recv;
        socket.unacked = info.tcpi_unacked;
        socket.byte_counters_valid = length >= offsetof(ExtendedInfo, bytes_sent) + sizeof(extended.bytes_sent);
        if (socket.byte_counters_valid)
        {
          socket.bytes_sent = extended.bytes_sent;
          socket.bytes_acked = extended.bytes_acked;
          socket.bytes_received = extended.bytes_received;
          socket.notsent_bytes = extended.notsent_bytes;
        }
      }
    }
    result.valid = true;
    return result;
  }

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
