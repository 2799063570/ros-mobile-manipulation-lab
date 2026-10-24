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
    for (int i = 0; i < 3; ++i)
      clients.push_back(sockets.client(listener, robot_endpoint));
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

    // Simulate an SDK disconnect and fd reuse by an unrelated connection.
    // The helper must refuse it without modifying or closing the new socket.
    int reused = clients.front();
    require(close(reused) == 0, "close failed");
    require(dup2(unrelated, reused) == reused, "dup2 failed");
    require(!transport.prepare(), "Disconnected/reused descriptor accepted");
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
