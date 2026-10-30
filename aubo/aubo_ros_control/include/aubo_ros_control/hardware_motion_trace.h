#pragma once

#include <aubo_ros_control/sdk_tcp_transport.h>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

namespace aubo_ros_control
{
// Capture only: control/feeder threads append into preallocated memory. Dump
// only after both producers have stopped. No motion or flow credits come from
// these records. The bounded files explicitly report dropped records.
class HardwareMotionTrace
{
public:
  using Clock = std::chrono::steady_clock;
  using TcpSnapshot = visual_servo_internal::SdkTcpTransport::Snapshot;
  struct Command
  {
    std::uint64_t id{0}, ros_ns{0}, sample_ns{0};
    std::int64_t steady_ns{0};
    bool seed{false};
    std::array<double, 6> q{};
  };
  struct Rpc
  {
    enum class Kind { MacQuery, Send, FeedbackQuery };
    Kind kind{Kind::MacQuery};
    std::uint64_t id{0}, first_point_id{0}, ros_ns{0};
    std::int64_t begin_ns{0}, end_ns{0};
    int count{0}, mac_scalars{-1}, result{0};
    std::uint16_t port_hint{0}; // New socket observed during the corresponding SDK login.
    TcpSnapshot before{}, after{};
  };
  explicit HardwareMotionTrace(const std::string& prefix, std::size_t capacity)
    : prefix_(prefix), commands_(checkedCapacity(prefix, capacity)), calls_(std::max<std::size_t>(64, capacity / 8))
  {
    for (const auto* suffix : {".commands.csv", ".rpc.csv"})
    {
      const auto path = prefix_ + suffix;
      const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (fd < 0) throw std::runtime_error("Cannot create NEW hardware trace: " + path);
      const std::string marker = "# incomplete: waiting for orderly hardware shutdown\n";
      const auto written = ::write(fd, marker.data(), marker.size());
      close(fd);
      if (written != static_cast<ssize_t>(marker.size())) throw std::runtime_error("Cannot initialize hardware trace");
    }
  }
  static std::int64_t ns()
  { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
  void append(const Command& event)
  { const auto i = command_count_.fetch_add(1); if (i < commands_.size()) commands_[i] = event; }
  void append(const Rpc& event)
  { const auto i = call_count_.fetch_add(1); if (i < calls_.size()) calls_[i] = event; }
  bool dumpAfterJoin() const
  {
    const auto points_path = prefix_ + ".commands.csv";
    const auto rpc_path = prefix_ + ".rpc.csv";
    const auto points_temp = points_path + ".export";
    const auto rpc_temp = rpc_path + ".export";
    // Leave the incomplete placeholders intact if killed during export.
    for (const auto& path : {points_temp, rpc_temp})
    {
      const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (fd < 0) return false;
      close(fd);
    }
    std::ofstream points(points_temp), rpc(rpc_temp);
    if (!points || !rpc) return false;
    points.imbue(std::locale::classic()); rpc.imbue(std::locale::classic());
    points << "# hardware_commands_v1; period_s=0.005; dropped=" << dropped(command_count_, commands_.size()) << '\n';
    points << "id,kind,steady_ns,ros_ns,sample_ns,q0,q1,q2,q3,q4,q5\n" << std::setprecision(17);
    for (std::size_t i = 0; i < std::min(command_count_.load(), commands_.size()); ++i)
    {
      const auto& e = commands_[i];
      points << e.id << ',' << (e.seed ? "seed" : "command") << ',' << e.steady_ns << ',' << e.ros_ns << ',' << e.sample_ns;
      for (double q : e.q) points << ',' << q;
      points << '\n';
    }
    rpc << "# hardware_rpc_v1; dropped=" << dropped(call_count_, calls_.size()) << '\n';
    rpc << "kind,id,first_point_id,ros_ns,begin_ns,end_ns,count,mac_scalars,result,port_hint,phase,tcp_valid,fd,local_port,bytes_valid,bytes_sent,bytes_acked,bytes_received,retransmissions,rtt_us,rto_us,last_data_sent_ms,last_data_received_ms,unacked,notsent_bytes\n";
    for (std::size_t i = 0; i < std::min(call_count_.load(), calls_.size()); ++i)
    {
      const auto& e = calls_[i];
      for (const bool after : {false, true})
      {
        const auto& tcp = after ? e.after : e.before;
        for (std::size_t j = 0; j < std::max<std::size_t>(1, tcp.detail_count); ++j)
        {
          const auto socket = j < tcp.detail_count ? tcp.sockets[j] : visual_servo_internal::SdkTcpTransport::SocketSample{};
          const char* kind = e.kind == Rpc::Kind::Send ? "send" :
              (e.kind == Rpc::Kind::FeedbackQuery ? "feedback_query" : "query");
          rpc << kind << ',' << e.id << ',' << e.first_point_id << ',' << e.ros_ns << ','
              << e.begin_ns << ',' << e.end_ns << ',' << e.count << ',' << e.mac_scalars << ',' << e.result << ',' << e.port_hint << ','
              << (after ? "after" : "before") << ',' << tcp.valid << ',' << socket.fd << ',' << socket.local_port << ','
              << socket.byte_counters_valid << ',' << socket.bytes_sent << ',' << socket.bytes_acked << ',' << socket.bytes_received << ','
              << socket.retransmissions << ',' << socket.rtt_us << ',' << socket.rto_us << ',' << socket.last_data_sent_ms << ','
              << socket.last_data_received_ms << ',' << socket.unacked << ',' << socket.notsent_bytes << '\n';
        }
      }
    }
    points.flush(); rpc.flush();
    points.close(); rpc.close();
    if (!points.good() || !rpc.good()) return false;
    return std::rename(points_temp.c_str(), points_path.c_str()) == 0 &&
           std::rename(rpc_temp.c_str(), rpc_path.c_str()) == 0;
  }
private:
  static std::size_t checkedCapacity(const std::string& prefix, std::size_t capacity)
  {
    if (prefix.empty() || prefix.front() != '/' || capacity == 0 || capacity > 1000000)
      throw std::invalid_argument("Hardware trace needs an absolute prefix and 1..1000000 command records");
    return capacity;
  }
  static std::size_t dropped(const std::atomic<std::size_t>& count, std::size_t capacity)
  { return count.load() > capacity ? count.load() - capacity : 0; }
  std::string prefix_;
  std::vector<Command> commands_;
  std::vector<Rpc> calls_;
  std::atomic<std::size_t> command_count_{0}, call_count_{0};
};
}
