#include <aubo_ros_control/cartesian_jog_common.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

using aubo_ros_control::jog::VelocitySmoother;

namespace
{
void require(bool value, const char* message)
{
  if (!value)
    throw std::runtime_error(message);
}

void derivatives(double velocity_limit, double acceleration_limit, double jerk_limit)
{
  constexpr double dt = 0.005;
  VelocitySmoother filter(velocity_limit, acceleration_limit, jerk_limit, dt);
  std::mt19937 random(42);
  std::uniform_real_distribution<double> targets(-2 * velocity_limit, 2 * velocity_limit);
  double last_velocity = 0, last_acceleration = 0;
  for (int i = 0; i < 100000; ++i)
  {
    // 包含长时间匀速、突然松手、直接反向、逐周期随机输入和 NaN。
    double target = i < 1000 ? velocity_limit : i < 2000 ? -velocity_limit : targets(random);
    if (i >= 2000 && i < 4000) target = 0;
    if (i % 113 == 0) target = std::numeric_limits<double>::quiet_NaN();
    const double velocity = filter.step(target);
    const double acceleration = (velocity - last_velocity) / dt;
    const double jerk = (acceleration - last_acceleration) / dt;
    require(std::isfinite(velocity), "Non-finite output");
    require(std::abs(velocity) <= velocity_limit + 1e-10, "Velocity bound exceeded");
    require(std::abs(acceleration) <= acceleration_limit + 1e-9, "Acceleration bound exceeded");
    require(std::abs(jerk) <= jerk_limit + 1e-7, "Jerk bound exceeded");
    if (i % 59 == 0)
    {
      auto braking = filter;
      double distance = 0;
      for (int j = 0; j < 5000; ++j)
        distance += std::abs(braking.step(0)) * dt;
      require(distance <= filter.stoppingDistance() + 1e-9, "Stopping-distance bound too small");
      require(braking.settled(), "Zero command did not settle");
    }
    last_velocity = velocity;
    last_acceleration = acceleration;
  }
  for (int i = 0; i < 600; ++i) filter.step(0);
  require(filter.settled(), "Default stop horizon exceeded");
}
}

int main()
{
  try
  {
    using aubo_ros_control::jog::interpolationSteps;
    require(interpolationSteps(50.0, 0.005) == 4, "50Hz must cover four 5ms points");
    require(interpolationSteps(100.0, 0.005) == 2, "100Hz must cover two 5ms points");
    require(interpolationSteps(200.0, 0.005) == 1, "200Hz must cover one 5ms point");
    for (double rate : {0.0, -1.0, 60.0, 201.0, 10.0, std::numeric_limits<double>::quiet_NaN()})
    {
      bool rejected_rate = false;
      try { interpolationSteps(rate, 0.005); }
      catch (const std::invalid_argument&) { rejected_rate = true; }
      require(rejected_rate, "Nonintegral/invalid control rate was accepted");
    }
    using aubo_ros_control::jog::dlsDamping;
    require(std::abs(dlsDamping(.008,.008,.06,.02,.002,true)-.02)<1e-12, "Near-stop damping must be preserved");
    require(std::abs(dlsDamping(.10,.008,.06,.02,.002,true)-.002)<1e-12, "Well-conditioned damping floor");
    require(std::abs(dlsDamping(.10,.008,.06,.02,.002,false)-.02)<1e-12, "Fixed damping compatibility");
    double previous=.02;
    for(int i=0;i<=1000;++i) {
      const double sigma=.008+.1*i/1000;
      const double lambda=dlsDamping(sigma,.008,.06,.02,.002,true);
      require(lambda<=previous+1e-12 && lambda>=.002 && lambda<=.02, "Adaptive damping bounds / continuity");
      previous=lambda;
    }
    const double sigma=.08;
    const double fixed_gain=sigma*sigma/(sigma*sigma+.02*.02);
    const double adaptive_gain=sigma*sigma/(sigma*sigma+.002*.002);
    require(1-adaptive_gain < .02*(1-fixed_gain), "Adaptive DLS must reduce unconstrained model bias");
    derivatives(0.25, 0.60, 8.0);
    derivatives(0.35, 0.80, 12.0);
    derivatives(0.05, 0.10, 100.0);  // alpha=1 分支。
    bool rejected = false;
    try { VelocitySmoother invalid(1, 0, 1, 0.005); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid parameters accepted");
    std::cout << "PASS: 300000 input steps; velocity, acceleration, jerk, braking bounds and settling\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
