#include <aubo_ros_control/direct_sdk_backend.h>

#include <iostream>
#include <limits>
#include <stdexcept>

namespace aubo_ros_control
{
namespace visual_servo_internal
{
struct DirectSdkBackendTestAccess
{
  static void require(bool value, const char* message)
  {
    if (!value)
      throw std::runtime_error(message);
  }

  static void run()
  {
    CommandQueue queue(2);
    JointPoint limits;
    limits.fill(1.0);
    DirectSdkBackend backend(queue, limits, limits, 200.0);
    // No login, startup, stream mode or output thread is used by this test.
    backend.connected_.store(true);
    JointPoint position{};
    require(!backend.readState(position), "Accepted missing feedback");
    aubo_robot_namespace::wayPoint_S sample{};
    for (std::size_t i = 0; i < kDof; ++i)
      sample.jointpos[i] = 0.1 * static_cast<double>(i + 1);
    backend.receiveWaypoint(nullptr, &backend);
    require(!backend.readState(position), "Accepted null feedback");
    backend.receiveWaypoint(&sample, &backend);
    require(backend.readState(position), "Rejected fresh feedback");
    require(backend.feedback_samples_ == 1, "Incorrect pushed sample counter");
    for (std::size_t i = 0; i < kDof; ++i)
      require(position[i] == sample.jointpos[i], "Incorrect joint ordering");
    const auto received = backend.feedback_received_;
    require(backend.readState(position), "Rejected repeated read of fresh feedback");
    require(backend.feedback_received_ == received, "Reading refreshed feedback age");
    backend.feedback_received_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    require(!backend.readState(position), "Accepted stale feedback");
    sample.jointpos[0] = std::numeric_limits<double>::quiet_NaN();
    backend.receiveWaypoint(&sample, &backend);
    require(!backend.readState(position), "Invalid sample refreshed stale feedback");
    require(backend.feedback_samples_ == 1, "Invalid sample counted as fresh feedback");
    sample.jointpos[0] = 0.1;
    backend.receiveWaypoint(&sample, &backend);
    require(backend.readState(position), "New sample did not restore fresh feedback");
    require(backend.feedback_samples_ == 2 && !backend.feedback_stale_reported_,
            "New sample did not reset stale diagnostic");
    backend.connected_.store(false);
    require(!backend.readState(position), "Accepted feedback after disconnect");
  }

  static void sendWatchdog()
  {
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::milliseconds;
    CommandQueue queue(2);
    JointPoint limits;
    limits.fill(1.0);
    for (const int target : {60, 300})
    for (const int late_result : {0, -1})
    {
      DirectSdkBackend backend(queue, limits, limits, 200.0);
      backend.mac_buffer_target_ = target;
      const int points = target / 6;
      const int budget_ms = target == 60 ? 50 : 100;
      const auto started = Clock::now();
      backend.point_generator_ = [](JointPoint&, PointTiming*) { return true; };
      backend.connected_.store(true);
      backend.running_.store(true);
      // A generous diagnosis TTL/capacity must not extend the configured per-call send budget.
      backend.flow_.reset(new SdkFlowControl(66, 0.5, started));
      const auto query = backend.flow_->beginQuery(started);
      require(backend.flow_->acceptReply(query, 0, started), "Initial diagnosis failed");
      require(backend.flow_->reserve(points, started), "Could not reserve pending batch");
      backend.io_diagnostics_.beginDiagnosis(started);
      backend.io_diagnostics_.finishDiagnosis(0, 0, started);
      const auto id = backend.io_diagnostics_.beginSend(points, 0.0001, 0.4, started);
      backend.send_started_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
          started.time_since_epoch()).count());
      require(backend.healthyAt(started + Ms(budget_ms - 1)), "Send stopped before configured budget");
      require(!backend.healthyAt(started + Ms(budget_ms)), "Over-budget pending send was accepted");
      require(!backend.running_.load() && !backend.connected_.load(), "Timeout did not stop backend");
      const auto reason = backend.failureReason();
      require(!reason.empty(), "Timeout lost original cause");
      require(backend.io_diagnostics_.finishSend(id, late_result, started + Ms(110)),
              "Late SDK return was not recorded");
      backend.send_started_ns_.store(0);
      const auto io = backend.io_diagnostics_.snapshot(started + Ms(111));
      require(io.last_send_valid && io.last_send_id == id && io.last_send_points == static_cast<std::size_t>(points) &&
              io.last_send_result == late_result && io.last_send_duration_ms == 110.0,
              "Late SDK return diagnostics incorrect");
      require(!backend.flow_->completeSend(points), "Late result revived stopped flow accounting");
      require(backend.flow_->inFlight() == static_cast<std::size_t>(points) && backend.flow_->upperBound() == static_cast<std::size_t>(points),
              "Timeout released in-flight reservations");
      backend.stopOutput("later secondary failure");
      require(backend.failureReason() == reason, "Secondary failure replaced original cause");
      require(!backend.healthyAt(started + Ms(111)), "Late SDK return revived backend");
    }
  }
  static void confirmationWatchdog()
  {
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::milliseconds;
    CommandQueue queue(2);
    JointPoint limits;
    limits.fill(1.0);
    for (const bool confirmed : {false, true})
    {
      DirectSdkBackend backend(queue, limits, limits, 200.0);
      const auto started = Clock::now();
      backend.point_generator_ = [](JointPoint&, PointTiming*) { return true; };
      backend.connected_.store(true);
      backend.running_.store(true);
      backend.flow_.reset(new SdkFlowControl(60, 0.5, started, SdkFlowControl::Policy::SequentialMac));
      const auto query = backend.flow_->beginQuery(started);
      require(backend.flow_->acceptReply(query, 0, started), "Confirmation setup query failed");
      backend.first_send_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(started.time_since_epoch()).count());
      backend.mac_confirmed_.store(confirmed);
      require(backend.healthyAt(started + Ms(299)), "Confirmation deadline fired before 300ms");
      require(backend.healthyAt(started + Ms(300)) == confirmed, "First MAC confirmation deadline incorrect");
      if (!confirmed) require(!backend.running_.load(), "Confirmation timeout did not latch stop");
    }
  }

};
}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

int main()
{
  try
  {
    aubo_ros_control::visual_servo_internal::DirectSdkBackendTestAccess::run();
    aubo_ros_control::visual_servo_internal::DirectSdkBackendTestAccess::sendWatchdog();
    aubo_ros_control::visual_servo_internal::DirectSdkBackendTestAccess::confirmationWatchdog();
    std::cout << "PASS: pushed feedback freshness; 49/50 and 99/100 ms send watchdogs, late returns, latched cause and 299/300ms first-MAC confirmation\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
