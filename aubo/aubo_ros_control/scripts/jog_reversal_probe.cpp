// Offline only: execute the actual production smoother; no ROS or SDK linkage.
// Build: g++ -O2 -std=c++14 -I../include jog_reversal_probe.cpp -o /tmp/jog_reversal_probe
// Run: /tmp/jog_reversal_probe > /tmp/jog_reversal.csv
#include <aubo_ros_control/cartesian_jog_common.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
  constexpr double h = .005;
  std::cout << "group,time,target,velocity,acceleration,jerk,position\n" << std::setprecision(17);
  for (int group = 0; group < 2; ++group)
  {
    const double V = group ? .35 : .25, A = group ? .8 : .6, J = group ? 12 : 8;
    aubo_ros_control::jog::VelocitySmoother filter(V, A, J, h);
    double previous_v = 0, previous_a = 0, position = 0;
    for (int i = 0; i < 1600; ++i)
    {
      const double time = i * h;
      const double target = time < 2 ? -V : time < 5 ? V : 0;
      const double v = filter.step(target), a = (v - previous_v) / h, j = (a - previous_a) / h;
      if (std::abs(v) > V+1e-9 || std::abs(a) > A+1e-9 || std::abs(j) > J+1e-7) return 1;
      position += v * h;
      std::cout << group << ',' << time << ',' << target << ',' << v << ',' << a << ',' << j << ',' << position << '\n';
      previous_v = v; previous_a = a;
    }
  }
}
