#include <aubo_ros_control/sdk_io_diagnostics.h>

#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>

using aubo_ros_control::visual_servo_internal::SdkIoDiagnostics;
using Time = SdkIoDiagnostics::Time;

namespace
{
Time at(int ms) { return Time{} + std::chrono::milliseconds(ms); }
void require(bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error(message);
}
}

int main()
{
  try
  {
    SdkIoDiagnostics io;
    auto s = io.snapshot(at(0));
    require(!s.send_active && !s.last_send_valid && !s.mac_valid &&
            !s.diagnosis_active && !s.last_diagnosis_valid,
            "Initial zero values were presented as real samples");

    const auto first = io.beginSend(10, 0.003, 0.4, at(0));
    require(first == 1, "First send did not receive ID 1");
    s = io.snapshot(at(50));
    require(s.send_active && s.active_send_id == first && s.active_send_points == 10 &&
            s.active_send_elapsed_ms == 50 && s.active_send_max_delta == 0.003 &&
            s.active_send_generation_ms == 0.4 && s.started_sends == 1 && s.returned_sends == 0,
            "Blocked first SDK call was not observable");
    require(io.beginSend(4, 0, 0, at(51)) == 0 && !io.finishSend(2, 0, at(52)),
            "Overlapping/mismatched batch corrupted active call");
    // A caller may already have stopped at 50ms. Telemetry still records the
    // eventual return, without knowing or changing the caller's fault state.
    require(io.finishSend(first, 0, at(105)), "Late SDK return was discarded");
    s = io.snapshot(at(110));
    require(!s.send_active && s.last_send_valid && s.last_send_id == first &&
            s.last_send_points == 10 && s.last_send_duration_ms == 105 &&
            s.last_send_result == 0 && s.last_send_max_delta == 0.003 &&
            s.last_send_generation_ms == 0.4 && s.started_sends == s.returned_sends,
            "Late SDK return lost metadata");
    require(!io.finishSend(first, -5, at(111)), "Duplicate return was counted twice");
    const auto second = io.beginSend(4, 0.001, 0.2, at(120));
    require(io.finishSend(second, -7, at(180)), "Failed SDK return was discarded");
    s = io.snapshot(at(181));
    require(s.last_send_id == 2 && s.last_send_result == -7 && s.last_send_duration_ms == 60 &&
            s.started_sends == 2 && s.returned_sends == 2, "Failed-send metadata was lost");

    require(io.beginDiagnosis(at(10)), "Diagnosis did not start");
    s = io.snapshot(at(60));
    require(s.diagnosis_active && s.diagnosis_elapsed_ms == 50 && !s.mac_valid,
            "Pending initial diagnosis looked like valid MAC zero");
    require(io.finishDiagnosis(0, 36, at(115)), "Diagnosis completion failed");
    s = io.snapshot(at(120));
    require(!s.diagnosis_active && s.mac_valid && s.mac_scalars == 36 &&
            s.mac_response_age_ms == 5 && s.mac_request_age_ms == 110 &&
            s.mac_diagnosis_duration_ms == 105, "MAC sample age interval is incorrect");
    require(io.beginDiagnosis(at(125)) && io.finishDiagnosis(-3, 0, at(135)),
            "Failed diagnosis was discarded");
    s = io.snapshot(at(140));
    require(s.last_diagnosis_valid && s.last_diagnosis_result == -3 &&
            s.last_diagnosis_duration_ms == 10 && s.mac_valid && s.mac_scalars == 36 &&
            s.mac_response_age_ms == 25 && s.mac_request_age_ms == 130 &&
            s.mac_diagnosis_duration_ms == 105,
            "Failed diagnosis replaced the last successful MAC observation");
    s = io.snapshot(at(0));
    require(s.mac_response_age_ms == 0 && s.mac_request_age_ms == 0,
            "Preempted snapshot exposed negative age");

    // Real concurrent updates and reads, no sleeps or SDK/robot dependencies.
    SdkIoDiagnostics concurrent;
    std::atomic<bool> finished{false}, consistent{true};
    std::thread sender([&] {
      for (int i = 0; i < 2000; ++i)
      {
        const auto id = concurrent.beginSend(4, 0.001, 0.2, at(i));
        if (!id || !concurrent.finishSend(id, 0, at(i + 1)))
          consistent.store(false);
      }
      finished.store(true);
    });
    std::thread diagnosis([&] {
      for (int i = 0; i < 2000; ++i)
        if (!concurrent.beginDiagnosis(at(i)) || !concurrent.finishDiagnosis(0, 48, at(i + 1)))
          consistent.store(false);
    });
    do
    {
      const auto view = concurrent.snapshot(at(3000));
      if (view.returned_sends > view.started_sends ||
          (view.send_active && (view.active_send_id != view.started_sends ||
           view.active_send_points != 4 || view.started_sends - view.returned_sends != 1)) ||
          (!view.send_active && view.started_sends != view.returned_sends))
        consistent.store(false);
    } while (!finished.load());
    sender.join();
    diagnosis.join();
    require(consistent.load(), "Concurrent snapshot mixed different batches");
    require(concurrent.snapshot(at(3000)).returned_sends == 2000,
            "Concurrent send returns were lost");
    const auto history = concurrent.history();
    require(history.size() == 64, "I/O history was not bounded");
    for (const auto& event : history)
    {
      require(event.result == 0 && event.duration_ms == 1, "History lost RPC result/duration");
      require(event.diagnosis ? event.count == 48 : event.count == 4, "History mixed scalar/point counts");
    }
    SdkIoDiagnostics ordered;
    for (int i = 0; i < 100; ++i)
    {
      const auto id = ordered.beginSend(i + 1, 0.001, 0.2, at(i * 2));
      require(ordered.finishSend(id, 0, at(i * 2 + 1)), "History setup send failed");
    }
    const auto ordered_history = ordered.history();
    require(ordered_history.size() == 64 && ordered_history.front().send_id == 37 &&
            ordered_history.back().send_id == 100, "History wraparound lost the most recent calls");

    std::cout << "PASS: first/blocked/late/failed SDK returns, valid MAC ages, "
                 "failed diagnosis retention, concurrent snapshots and bounded 64-call history\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
