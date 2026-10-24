#include <aubo_ros_control/motion_timing_trace.h>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>
#include <vector>
#include <cstdio>
#include <unistd.h>

using aubo_ros_control::visual_servo_internal::MotionTimingTrace;

int main()
{
  const std::string path = "/tmp/aubo_motion_trace_test_" + std::to_string(getpid()) + ".csv";
  MotionTimingTrace trace(path, 4000);
  std::vector<std::thread> writers;
  for (unsigned thread = 0; thread < 4; ++thread)
    writers.emplace_back([&, thread] {
      for (unsigned n = 0; n < 1000; ++n)
      {
        MotionTimingTrace::Event event;
        event.id = thread * 1000 + n + 1;
        event.begin_ns = event.id * 100;
        event.end_ns = event.begin_ns + 90;
        event.q[5] = event.id;
        trace.append(event);
      }
    });
  for (auto& writer : writers) writer.join();
  MotionTimingTrace::Event overflow;
  for (int n = 0; n < 13; ++n) trace.append(overflow);
  if (!trace.dumpAfterJoin() || trace.droppedAfterJoin() != 13) return 1;
  std::ifstream input(path);
  std::string line;
  std::getline(input, line); std::getline(input, line);
  std::set<unsigned> ids;
  while (std::getline(input, line))
  {
    std::istringstream row(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(row, field, ',')) fields.push_back(field);
    if (fields.size() != 31) return 2;
    const auto id = std::stoul(fields[1]);
    if (std::stoll(fields[6]) != static_cast<long long>(id * 100) ||
        std::stod(fields[18]) != id || !ids.insert(id).second) return 3;
  }
  std::remove(path.c_str());
  if (ids.size() != 4000 || *ids.begin() != 1 || *ids.rbegin() != 4000) return 4;
  std::cout << "PASS: concurrent append, joined dump, point IDs, bounded overflow\n";
}
