#include <aubo_ros_control/visual_servo_common.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace vsi = aubo_ros_control::visual_servo_internal;

void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

int main()
{
  using State = vsi::ServoState;
  for (bool enabled : {false, true}) {
    for (bool fresh : {false, true}) {
      require(vsi::selectServoState(enabled, true, fresh) == State::FAULT,
              "fault must dominate enable and target validity");
    }
  }
  require(vsi::selectServoState(false, false, true) == State::DISABLED,
          "targets must not enable motion");
  require(vsi::selectServoState(true, false, false) == State::HOLD,
          "target loss must hold without autonomous search");
  require(vsi::selectServoState(true, false, true) == State::TRACKING,
          "reacquisition must resume tracking without a search dwell");

  vsi::AlignmentTracker arrival;
  require(!arrival.update(true, 0.0, 0.35), "arrival needs dwell");
  require(!arrival.update(true, 0.34, 0.35), "arrival dwell too short");
  require(arrival.update(true, 0.36, 0.35), "arrival should latch");

  // A 10 mm error is inside the 12 mm reporting hysteresis but outside the
  // 6 mm servo deadband. It must still request positive correction.
  const double error = 0.010, deadband = 0.006;
  require(arrival.update(error <= deadband * 2.0, 0.4, 0.35),
          "arrival hysteresis should remain set");
  require(vsi::selectServoState(true, false, true) == State::TRACKING,
          "arrival must not disable tracking");
  require(std::abs(0.45 * vsi::applyDeadband(error, deadband) - 0.0018) < 1e-12,
          "correction inside arrival hysteresis must remain nonzero");
  require(!arrival.update(false, 0.5, 0.35), "leaving window clears arrival");
  require(!arrival.update(true, 0.6, 0.35), "reentry needs a new dwell");
  arrival.reset();  // Used on target loss, disable, reset and reconfiguration.
  require(!arrival.aligned(), "reset must clear arrival");
  require(!arrival.update(true, 1.0, 0.35), "loss breaks consecutive dwell");
  require(arrival.update(true, 1.4, 0.35), "new dwell can complete");
  require(!arrival.update(true, 1.2, 0.35),
          "rewind after candidate start must clear a latched arrival");
  require(!arrival.update(true, 1.5, 0.35), "rewind must restart the full dwell");
  require(arrival.update(true, 1.6, 0.35), "arrival can recover after rewind");
  require(!arrival.update(true, 0.2, 0.35), "clock rewind restarts dwell");
  require(!arrival.update(true, std::numeric_limits<double>::quiet_NaN(), 0.35),
          "invalid clock must not report arrival");
  require(arrival.update(true, 2.0, 0.0), "zero dwell should be immediate");

  std::cout << "Servo policy regression checks passed\n";
}
