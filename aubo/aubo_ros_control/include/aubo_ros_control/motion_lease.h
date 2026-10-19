#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace aubo_ros_control
{

// 同一台 Linux 主机上，点动、视觉伺服和 ros_control 只能有一个运动写入者。
// 文件锁在进程崩溃后由内核释放；不删除锁文件，避免 inode 更换导致双重持锁。
// 此锁不能约束其他电脑、示教器或仓库外 SDK 程序。
class MotionLease
{
public:
  MotionLease() = default;
  MotionLease(const MotionLease&) = delete;
  MotionLease& operator=(const MotionLease&) = delete;
  ~MotionLease() { release(); }

  bool acquire()
  {
    if (fd_ >= 0)
      return true;
    // 本工程只有一台臂；固定文件名也避免 IP 别名绕过互斥。
    const int descriptor = ::open("/tmp/aubo_motion_writer.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0666);
    if (descriptor < 0)
      return false;
    if (::flock(descriptor, LOCK_EX | LOCK_NB) != 0)
    {
      ::close(descriptor);
      return false;
    }
    fd_ = descriptor;
    return true;
  }

  void release()
  {
    if (fd_ >= 0)
    {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_{-1};
};

}  // namespace aubo_ros_control
