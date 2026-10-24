#include <aubo_ros_control/sdk_flow_control.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

using aubo_ros_control::visual_servo_internal::SdkFlowControl;
using Fault = SdkFlowControl::Fault;
using TimePoint = SdkFlowControl::TimePoint;

namespace
{
TimePoint at(int milliseconds)
{
  return TimePoint{} + std::chrono::milliseconds(milliseconds);
}

void require(bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error(message);
}

void initialize(SdkFlowControl& flow, std::size_t raw_scalars = 0)
{
  const auto query = flow.beginQuery(at(0));
  require(static_cast<bool>(query), "Missing initial query token");
  require(flow.acceptReply(query, raw_scalars, at(0)), "Initial diagnosis rejected");
}

// Model real buffer consumption, a single diagnostic connection, and a paced
// sender (by default two points per ten milliseconds). Every slow reply reports the
// EARLIEST possible snapshot (at request start), not a convenient fresh count.
bool runSlowQueries(std::size_t cap, bool consuming,
                    std::size_t batch_points = 2, int first_refill = 10,
                    int refill_period = 10, int query_latency = 100)
{
  SdkFlowControl flow(cap, 0.15, at(0));
  initialize(flow);
  require(flow.reserve(10, at(0)), "Initial preload rejected");
  require(flow.completeSend(10), "Initial preload completion rejected");
  std::size_t actual = 10;
  auto query = flow.beginQuery(at(1));
  std::size_t snapshot = actual;
  int request_start = 1;
  int next_query = -1;
  std::size_t max_actual = actual;
  for (int ms = 2; ms <= 2500; ++ms)
  {
    if (consuming && ms % 5 == 0 && actual != 0)
      --actual;
    if (query && ms == request_start + query_latency)
    {
      require(flow.acceptReply(query, snapshot * 6, at(ms)), "Slow diagnosis reply rejected");
      query = {};
      next_query = ms + 5;
    }
    if (!query && ms == next_query)
    {
      query = flow.beginQuery(at(ms));
      snapshot = actual;
      request_start = ms;
    }
    if (ms >= first_refill && (ms - first_refill) % refill_period == 0)
    {
      if (!flow.reserve(batch_points, at(ms)))
      {
        require(flow.fault() == Fault::Capacity, "Unexpected paced-send fault");
        require(actual <= cap && flow.upperBound() <= cap,
                "Hard cap exceeded before rejection");
        return false;
      }
      // Exercise the window between reservation and acknowledgement too.
      require(actual + batch_points <= flow.upperBound(), "Reservation omitted from upper bound");
      actual += batch_points;
      require(flow.completeSend(batch_points), "Paced-send completion rejected");
      max_actual = std::max(max_actual, actual);
    }
    require(flow.healthy(at(ms)), "Slow queries tripped 150ms watchdog");
    require(actual <= flow.upperBound(), "Ledger underestimated actual occupancy");
    require(flow.upperBound() <= cap, "Ledger exceeded configured hard cap");
    if (consuming)
      require(actual != 0, "Normal consumption ran dry");
  }
  require(consuming && max_actual <= 12, "Stopped consumer was not bounded");
  return true;
}
}

int main()
{
  try
  {
    // A slow diagnosis cannot return credit merely because time passed.
    SdkFlowControl no_clock_credit(60, 0.15, at(0));
    require(!no_clock_credit.reserve(1, at(0)), "Uninitialized stream allowed sends");
    initialize(no_clock_credit);
    require(no_clock_credit.reserve(10, at(1)), "Preload failed");
    require(no_clock_credit.completeSend(10), "Send completion failed");
    require(no_clock_credit.healthy(at(100)), "Freshness failed before timeout");
    require(no_clock_credit.upperBound() == 10, "Elapsed time fabricated consumption");

    // An initial nonempty snapshot and partial six-scalar point count are
    // conservatively retained, rather than assuming a newly connected queue is empty.
    SdkFlowControl nonempty(60, 0.15, at(0));
    initialize(nonempty, 7);
    require(nonempty.upperBound() == 2, "Partial point did not round up");

    // Query started before the first send: its old zero must retain all later
    // reservations, even after those sends have completed on the other socket.
    SdkFlowControl old_zero(60, 0.15, at(0));
    initialize(old_zero);
    const auto before_send = old_zero.beginQuery(at(1));
    require(old_zero.reserve(10, at(2)), "Concurrent send reservation failed");
    require(old_zero.completeSend(10), "Concurrent send completion failed");
    require(old_zero.acceptReply(before_send, 0, at(100)), "Old zero rejected");
    require(old_zero.upperBound() == 10, "Old zero erased newer submissions");

    // Most important cross-thread race: a send reserved BEFORE query start is
    // still in flight, so its eventual arrival may not be in this zero snapshot.
    SdkFlowControl in_flight(60, 0.15, at(0));
    initialize(in_flight);
    require(in_flight.reserve(10, at(1)), "Inflight reservation failed");
    const auto during_send = in_flight.beginQuery(at(2));
    require(in_flight.acceptReply(during_send, 0, at(3)), "Inflight zero rejected");
    require(in_flight.upperBound() == 10 && in_flight.inFlight() == 10,
            "Query lost already-reserved inflight points");
    require(in_flight.completeSend(10), "Inflight completion failed");
    require(in_flight.upperBound() == 10, "Completion returned unobserved credits");
    const auto after_send = in_flight.beginQuery(at(4));
    require(in_flight.acceptReply(after_send, 0, at(5)), "Fresh drained snapshot rejected");
    require(in_flight.upperBound() == 0, "Fresh snapshot failed to return credits");
    require(in_flight.healthy(at(4)),
            "Caller preempted before mutex spuriously rejected a newer reply");

    require(runSlowQueries(60, true), "300ms cap failed repeated 100ms diagnosis");
    require(!runSlowQueries(40, true),
            "200ms cap unexpectedly sustained conservative 100ms snapshots");
    require(!runSlowQueries(60, false), "Paused consumer allowed unlimited queue growth");

    // Application pacing: keep the initial ten points, first refill at 30ms,
    // then send four points every 20ms. A 105ms RPC plus a 5ms query gap must
    // not empty the real buffer or make the conservative bound exceed 60.
    require(runSlowQueries(60, true, 4, 30, 20, 105),
            "Four-point pacing failed repeated 105ms diagnosis");
    require(!runSlowQueries(60, false, 4, 30, 20, 105),
            "Four-point pacing allowed stopped-consumer queue growth");

    SdkFlowControl delayed_zero(60, 0.15, at(0));
    initialize(delayed_zero);
    const auto delayed_query = delayed_zero.beginQuery(at(1));
    require(delayed_zero.reserve(10, at(2)) && delayed_zero.completeSend(10),
            "Delayed-zero initial send failed");
    for (int ms = 30; ms <= 90; ms += 20)
      require(delayed_zero.reserve(4, at(ms)) && delayed_zero.completeSend(4),
              "Delayed-zero paced send failed");
    require(delayed_zero.acceptReply(delayed_query, 0, at(106)),
            "105ms old-zero reply rejected");
    require(delayed_zero.upperBound() == 26,
            "105ms old zero erased submissions after request start");

    // One 300ms RPC stalls independently of the sender; the sender's watchdog
    // must reject at 150ms, and the late result must never revive the stream.
    SdkFlowControl blocked(60, 0.15, at(0));
    initialize(blocked);
    const auto blocked_query = blocked.beginQuery(at(5));
    require(blocked.healthy(at(149)), "Watchdog fired early");
    require(!blocked.healthy(at(150)), "Dead poller exceeded receive-time watchdog");
    require(!blocked.acceptReply(blocked_query, 0, at(305)), "Late reply revived failed stream");
    require(!blocked.reserve(1, at(306)), "Failed stream resumed sending");

    // A completed reply cannot hide an overlong request, even if no sender
    // happened to check the watchdog during that interval.
    SdkFlowControl slow_reply(60, 0.15, at(0));
    initialize(slow_reply);
    const auto too_slow = slow_reply.beginQuery(at(1));
    require(!slow_reply.acceptReply(too_slow, 0, at(301)), "Overlong RPC was accepted");

    SdkFlowControl send_failure(60, 0.15, at(0));
    initialize(send_failure);
    require(send_failure.reserve(10, at(1)), "Failed-send reservation failed");
    const auto failure_query = send_failure.beginQuery(at(2));
    send_failure.sendFailed();
    require(!send_failure.completeSend(10), "Failed send acknowledged later");
    require(send_failure.inFlight() == 10 && send_failure.upperBound() == 10,
            "Ambiguous failed send returned credits");
    require(!send_failure.acceptReply(failure_query, 0, at(3)),
            "Diagnosis revived ambiguous failed send");

    SdkFlowControl overflow(60, 0.15, at(0));
    const auto too_full = overflow.beginQuery(at(0));
    require(!overflow.acceptReply(too_full, 361, at(1)), "Actual queue beyond cap accepted");
    require(overflow.fault() == Fault::Capacity, "Wrong overflow failure");

    SdkFlowControl duplicate(60, 0.15, at(0));
    initialize(duplicate);
    const auto completed_query = duplicate.beginQuery(at(1));
    require(duplicate.acceptReply(completed_query, 0, at(2)), "Valid query rejected");
    require(!duplicate.acceptReply(completed_query, 0, at(3)), "Duplicate reply accepted");

    SdkFlowControl unexpected(60, 0.15, at(0));
    initialize(unexpected);
    require(unexpected.reserve(10, at(1)) && unexpected.completeSend(10), "Preload failed");
    const auto unexpected_query = unexpected.beginQuery(at(2));
    require(!unexpected.acceptReply(unexpected_query, 120, at(3)),
            "Unexpected observed data above accounting bound was silently ignored");

    SdkFlowControl stopped(60, 0.15, at(0));
    initialize(stopped);
    const auto cancelled_query = stopped.beginQuery(at(1));
    stopped.stop();
    require(!stopped.reserve(4, at(2)), "Stopped stream accepted a new batch");
    require(!stopped.acceptReply(cancelled_query, 0, at(3)), "Late reply revived stopped stream");

    SdkFlowControl bad_completion(60, 0.15, at(0));
    initialize(bad_completion);
    require(!bad_completion.completeSend(1), "Unreserved completion accepted");

    std::cout << "PASS: request-start snapshots, inflight reservations, no time credit, "
                 "repeated 100/105ms RPCs, two/four-point pacing, paused consumer, bounded stale RPC, "
                 "failed sends and protocol errors\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
