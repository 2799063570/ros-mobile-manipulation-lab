#include <aubo_ros_control/sdk_jog_timing.h>
#include <aubo_ros_control/sdk_flow_control.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

using aubo_ros_control::visual_servo_internal::SdkFlowControl;
using aubo_ros_control::visual_servo_internal::SdkJogTiming;
using aubo_ros_control::visual_servo_internal::SdkPointPacer;

namespace
{
void require(bool value, const char* reason)
{
  if (!value) throw std::runtime_error(reason);
}
SdkFlowControl::TimePoint at(int ms)
{
  return SdkFlowControl::TimePoint{} + std::chrono::milliseconds(ms);
}

// Same order as jogOutputLoop: query, generate, reserve, synchronous send,
// acknowledgement, next query. Snapshots use the EARLIEST request instant;
// batches may be accepted at either end of the synchronous send interval.
void stream(int send_ms, int diagnosis_ms, bool accept_late, bool consume = true,
            int cache_lag_ms = 0, int startup_delay_ms = 0, bool use_pacer = true,
            int second_send_ms = 0, bool relax_send_watchdog = false)
{
  const SdkJogTiming timing(300);
  SdkPointPacer pacer(timing.target_points);
  SdkFlowControl flow(60, 0.15, at(0), SdkFlowControl::Policy::SequentialMac);
  int now = 0, actual = 0, next_tick = 0, batches = 0;
  bool active = false, confirmed = false;
  std::vector<std::pair<int,int>> cabinet_history{{0,0}};
  auto advance = [&](int duration) {
    const int end = now + duration;
    while (active && consume && next_tick <= end)
    {
      require(actual > 0, "Cabinet starved during slow query/send");
      --actual;
      cabinet_history.push_back({next_tick, actual});
      next_tick += 5;
    }
    now = end;
    require(flow.healthy(at(now)), "Combined cycle exceeded diagnosis freshness");
    if (cache_lag_ms == 0)
      require(actual <= static_cast<int>(flow.upperBound()), "Fresh ledger underestimated occupancy");
    require(actual <= 60, "Physical model queue exceeded 300ms capacity");
    require(flow.upperBound() <= 60, "300ms hard cap exceeded");
  };
  auto accept = [&](int count) {
    actual += count;
    cabinet_history.push_back({now, actual});
    if (!active) { active = true; next_tick = now + startup_delay_ms + 5; }
    require(actual <= 50, "MAC target exceeded");
  };
  while (now < 30000)
  {
    const auto query = flow.beginQuery(at(now));
    require(static_cast<bool>(query), "Query rejected");
    int snapshot = actual;
    if (cache_lag_ms)
      for (auto i = cabinet_history.rbegin(); i != cabinet_history.rend(); ++i)
        if (i->first <= now - cache_lag_ms) { snapshot = i->second; break; }
    const int raw = snapshot * 6;
    advance(diagnosis_ms);
    require(flow.acceptReply(query, raw, at(now)), "Diagnosis rejected");
    if (active && raw > 0) confirmed = true;
    if (active && !confirmed) { advance(1); continue; }
    if (active) require(raw > 0, "Observed an empty established stream");
    const auto deficit = timing.wanted(raw);
    if (batches && !pacer.started()) pacer.start(at(now), deficit);
    if (batches && deficit == 0) pacer.discard(at(now));
    const auto wanted = batches && use_pacer ? pacer.allowance(deficit, at(now)) : deficit;
    if (wanted != 0)
    {
      advance(1);  // point generation is not free
      require(flow.reserve(wanted, at(now)), "Credit reservation failed");
      require(!batches || !use_pacer || pacer.reserve(wanted, at(now)), "Producer rate allowance missing");
      if (!accept_late) accept(wanted);
      // Replay the user's fast first send, then sustained slow sends.
      const int latency = batches == 0 ? (second_send_ms ? 22 : 17) :
          (batches == 1 && second_send_ms ? second_send_ms : send_ms);
      // Relaxation is a negative control only, never the production policy.
      const double budget_ms = relax_send_watchdog ? 130.0 : timing.sendBudget() * 1000;
      require(latency < budget_ms, "Send watchdog not respected");
      advance(latency);
      if (accept_late) accept(wanted);
      require(flow.completeSend(wanted), "Send completion rejected");
      ++batches;
    }
    advance(1);
  }
  require(batches > (consume ? 200 : 0), "No sustained stream exercised");
  if (!consume) require(batches == 1 && actual == 50, "Paused consumer grew the queue");
}
}

int main()
{
  try
  {
    const SdkJogTiming timing(300);
    require(timing.target_points == 50 && timing.lookahead() == 0.25, "Wrong 5ms point timing");
    require(timing.wanted(0) == 50 && timing.wanted(48) == 42, "MAC deficit calculation incorrect");
    require(timing.wanted(7) == 48, "Partial point not rounded conservatively");
    require(timing.wanted(300) == 0 && timing.wanted(360) == 0, "Overfull queue accepted");
    for (const int invalid : {0, 23, 25, 301, 360})
    {
      bool rejected = false;
      try { SdkJogTiming bad(invalid); } catch (const std::invalid_argument&) { rejected = true; }
      require(rejected, "Invalid target accepted");
    }
    for (const bool late : {false, true})
    {
      stream(60, 20, late);
      stream(89, 20, late);
      stream(90, 20, late);
      stream(60, 60, late);
      stream(89, 25, late);
      stream(60, 20, late, false);
      stream(60, 20, late, true, 20);
      stream(90, 20, late, true, 20);
      stream(60, 20, late, true, 20, 200);
    }
    bool old_overfed = false;
    try { stream(60, 20, true, true, 20, 0, false); }
    catch (const std::runtime_error& error)
    { old_overfed = std::string(error.what()) == "MAC target exceeded"; }
    require(old_overfed, "Negative control did not reproduce unpaced cached-MAC overfeeding");

    // 2026-10-24 X+: first send 21.73ms, second send 118.73ms,
    // latest diagnosis 23.95ms. Merely raising 100 -> 130ms cannot guarantee
    // continuity: late acceptance plus cached MAC can starve the next refill.
    for (const bool relaxed : {false, true})
    {
      bool expected_failure = false;
      try { stream(60, 24, true, true, 20, 0, true, 119, relaxed); }
      catch (const std::runtime_error& error)
      {
        expected_failure = std::string(error.what()) == (relaxed ?
            "Cabinet starved during slow query/send" : "Send watchdog not respected");
      }
      require(expected_failure, "119ms second-send negative control did not expose timeout-only workaround");
    }

    SdkPointPacer paced(50);
    paced.start(at(0));
    require(paced.allowance(50, at(19)) == 0, "Sent less than the initial 20ms quantum");
    require(paced.allowance(50, at(20)) == 4 && paced.reserve(4, at(20)), "Missing 200Hz production quota");
    require(paced.allowance(50, at(20)) == 0, "Repeated cached MAC created production quota");
    require(paced.allowance(50, at(110)) == 18, "Slow RPC did not accrue correct point quota");
    require(!paced.reserve(19, at(110)), "Producer exceeded fixed 5ms point rate");
    require(paced.allowance(50, at(10000)) == 50, "Unlimited production burst credit");
    paced.discard(at(10000));
    require(paced.allowance(50, at(10000)) == 0, "Full MAC retained stale production credit");

    // Replay the real failure: target 300 scalars, then MAC reports 306.
    // It must stop REFILLING, not terminate the node, while retaining 360 cap.
    SdkFlowControl observed(60, 0.15, at(0), SdkFlowControl::Policy::SequentialMac);
    auto observation = observed.beginQuery(at(0));
    require(observed.acceptReply(observation, 0, at(1)), "Initial observed query failed");
    require(observed.reserve(50, at(2)) && observed.completeSend(50), "Observed preload failed");
    observation = observed.beginQuery(at(3));
    require(observed.acceptReply(observation, 300, at(4)), "300-scalar baseline rejected");
    observation = observed.beginQuery(at(5));
    require(observed.acceptReply(observation, 306, at(6)), "Real 306-scalar observation falsely faulted");
    require(observed.upperBound() == 51 && timing.wanted(306) == 0, "Over-target MAC caused refill");
    observation = observed.beginQuery(at(7));
    require(observed.acceptReply(observation, 288, at(8)), "Later consumed observation rejected");
    require(timing.wanted(288) == 0 && timing.wanted(276) == 4 && observed.reserve(4, at(9)) && observed.completeSend(4),
            "Measured deficit did not resume bounded refill");
    observation = observed.beginQuery(at(10));
    require(observed.acceptReply(observation, 360, at(11)), "Exactly 60 observed points rejected");
    require(timing.wanted(360) == 0, "Hard-limit observation requested more points");
    observation = observed.beginQuery(at(12));
    require(!observed.acceptReply(observation, 361, at(13)), "Observed count beyond 60 points accepted");
    require(observed.fault() == SdkFlowControl::Fault::Capacity, "Wrong hard-limit failure");

    SdkFlowControl overlap(60, 0.15, at(0), SdkFlowControl::Policy::SequentialMac);
    observation = overlap.beginQuery(at(0));
    require(overlap.acceptReply(observation, 0, at(1)) && overlap.reserve(5, at(2)), "Overlap setup failed");
    require(!overlap.beginQuery(at(3)), "Sequential query overlapped an unacknowledged send");
    SdkFlowControl during_query(60, 0.15, at(0), SdkFlowControl::Policy::SequentialMac);
    observation = during_query.beginQuery(at(0));
    require(during_query.acceptReply(observation, 0, at(1)), "During-query setup failed");
    observation = during_query.beginQuery(at(2));
    require(!during_query.reserve(1, at(3)), "Sequential send overlapped diagnosis");

    // A send returning successfully after 100ms may not revive the stream.
    SdkFlowControl stopped(60, 0.15, at(0));
    const auto query = stopped.beginQuery(at(0));
    require(stopped.acceptReply(query, 0, at(20)) && stopped.reserve(50, at(21)), "Setup failed");
    stopped.stop();
    require(!stopped.completeSend(50) && !stopped.reserve(1, at(132)), "Late reply revived flow");
    require(stopped.inFlight() == 50 && stopped.upperBound() == 50, "Late reply released capacity");
    std::cout << "PASS: 30s streams at 60/89/90ms send, 20/25/60ms diagnosis, early/late acceptance, paused consumer, 300/306/360/361-scalar replay, serialized RPCs, 20ms cached MAC, 200ms startup delay, rate-limit negative control, 119ms second-send timeout-only negative control and late-result latch\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
