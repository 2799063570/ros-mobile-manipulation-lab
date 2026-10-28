#include <aubo_ros_control/sdk_tcp_transport.h>

#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool value, const char* message)
{
  if (!value)
    throw std::runtime_error(message);
}

struct Sockets
{
  std::vector<int> owned;
  int last_peer{-1};
  ~Sockets() { for (int fd : owned) close(fd); }
  int make(int type = SOCK_STREAM)
  {
    int fd = socket(AF_INET, type, 0);
    require(fd >= 0, "socket failed");
    owned.push_back(fd);
    return fd;
  }
  int listener(sockaddr_in& endpoint)
  {
    int fd = make();
    endpoint = {};
    endpoint.sin_family = AF_INET;
    endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    require(bind(fd, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) == 0,
            "bind failed");
    require(listen(fd, 4) == 0, "listen failed");
    socklen_t length = sizeof(endpoint);
    require(getsockname(fd, reinterpret_cast<sockaddr*>(&endpoint), &length) == 0,
            "getsockname failed");
    return fd;
  }
  int client(int listener, const sockaddr_in& endpoint)
  {
    int fd = make();
    require(connect(fd, reinterpret_cast<const sockaddr*>(&endpoint), sizeof(endpoint)) == 0,
            "connect failed");
    int peer = accept(listener, nullptr, nullptr);
    require(peer >= 0, "accept failed");
    owned.push_back(peer);
    last_peer = peer;
    return fd;
  }
};

int noDelay(int fd)
{
  int value = -1;
  socklen_t length = sizeof(value);
  require(getsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &value, &length) == 0,
          "getsockopt failed");
  return value;
}
}

int main()
{
  try
  {
    Sockets sockets;
    sockaddr_in robot_endpoint{}, other_endpoint{};
    int listener = sockets.listener(robot_endpoint);
    int other_listener = sockets.listener(other_endpoint);
    std::vector<int> clients;
    std::vector<int> peers;
    for (int i = 0; i < 3; ++i)
    {
      clients.push_back(sockets.client(listener, robot_endpoint));
      peers.push_back(sockets.last_peer);
    }
    int unrelated = sockets.client(other_listener, other_endpoint);
    int udp = sockets.make(SOCK_DGRAM);
    require(connect(udp, reinterpret_cast<sockaddr*>(&robot_endpoint), sizeof(robot_endpoint)) == 0,
            "UDP connect failed");

    aubo_ros_control::visual_servo_internal::SdkTcpTransport transport;
    require(!transport.configure("invalid", 8899), "Invalid address accepted");
    require(!transport.configure("127.0.0.1", 65536), "Invalid port accepted");
    require(!transport.configure("127.0.0.2", ntohs(robot_endpoint.sin_port)),
            "Unmatched endpoint accepted");
    for (int fd : clients)
      require(noDelay(fd) == 0, "Unexpected default TCP_NODELAY");
    require(transport.configure("127.0.0.1", ntohs(robot_endpoint.sin_port)),
            "Configure failed");
    require(transport.size() == 3, "Incorrect connection selection (TCP vs UDP/server/other port)");
    for (int fd : clients)
      require(noDelay(fd) == 1, "TCP_NODELAY was not enabled");
    require(noDelay(unrelated) == 0, "Unrelated connection modified");
    require(transport.prepare(), "QUICKACK refresh failed");
    const auto tcp = transport.snapshot();
    require(tcp.valid && tcp.connections == 3, "TCP_INFO did not select exactly the controller sockets");
    require(tcp.max_rto_us > 0, "TCP_INFO has no retransmission timeout");
    const auto find_socket = [&](const decltype(tcp)& sample, int fd) {
      for (std::size_t i = 0; i < sample.detail_count; ++i)
        if (sample.sockets[i].fd == fd) return sample.sockets[i];
      throw std::runtime_error("TCP socket details missing");
    };
    const auto before = find_socket(tcp, clients[0]);
    require(before.byte_counters_valid && before.local_port != 0, "Extended TCP_INFO unavailable");
    char bytes[6];
    require(send(clients[0], "abcdef", 6, MSG_NOSIGNAL) == 6 && recv(peers[0], bytes, 6, MSG_WAITALL) == 6,
            "Loopback request failed");
    require(send(peers[0], "ok", 2, MSG_NOSIGNAL) == 2 && recv(clients[0], bytes, 2, MSG_WAITALL) == 2,
            "Loopback reply failed");
    const auto after = find_socket(transport.snapshot(), clients[0]);
    require(after.bytes_sent >= before.bytes_sent + 6 && after.bytes_received >= before.bytes_received + 2,
            "Extended TCP_INFO byte counters do not match request/reply");

    // Simulate an SDK disconnect and fd reuse by an unrelated connection.
    // The helper must refuse it without modifying or closing the new socket.
    int reused = clients.front();
    require(close(reused) == 0, "close failed");
    require(dup2(unrelated, reused) == reused, "dup2 failed");
    require(!transport.prepare(), "Disconnected/reused descriptor accepted");
    require(!transport.snapshot().valid, "TCP_INFO accepted a reused unrelated descriptor");
    require(noDelay(reused) == 0, "Reused descriptor modified");
    require(send(unrelated, "x", 1, MSG_NOSIGNAL) == 1, "Unrelated connection closed");
    std::cout << "PASS: endpoint filtering, TCP_NODELAY, QUICKACK, invalid input and fd reuse\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
