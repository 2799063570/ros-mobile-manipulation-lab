// Test-only implementation of the SDK boundary. No SDK login/startup/stream
// function is called; this file is linked only into the non-installed test node.
#include <aubo_ros_control/direct_sdk_backend.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/topic.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

namespace aubo_ros_control
{
namespace visual_servo_internal
{
namespace
{
using Clock = std::chrono::steady_clock;
struct Transport
{
  ros::Publisher batches;
  ros::Subscriber feedback;
  int send_delay_ms{60}, diagnosis_delay_ms{20}, cache_lag_ms{20};
  bool accept_late{false};
};
std::unique_ptr<Transport> transport;
}

DirectSdkBackend::DirectSdkBackend(CommandQueue& queue, const JointPoint& velocity,
                                 const JointPoint& acceleration, double rate)
  : queue_(queue), velocity_limits_(velocity), acceleration_limits_(acceleration), period_(1.0 / rate)
{
  last_position_.fill(0.0);
  last_velocity_.fill(0.0);
}

DirectSdkBackend::~DirectSdkBackend() { shutdown(); }

void DirectSdkBackend::setPointGenerator(std::function<bool(JointPoint&, PointTiming*)> generator)
{ point_generator_ = std::move(generator); }

bool DirectSdkBackend::connect(ros::NodeHandle& nh)
{
  nh.param("feedback_timeout", feedback_timeout_, 0.15);
  transport.reset(new Transport);
  nh.param("sdk_mac_buffer_target", mac_buffer_target_, 300);
  nh.param("fake_send_delay_ms", transport->send_delay_ms, 60);
  nh.param("fake_diagnosis_delay_ms", transport->diagnosis_delay_ms, 20);
  nh.param("fake_mac_cache_lag_ms", transport->cache_lag_ms, 20);
  nh.param("fake_accept_late", transport->accept_late, false);
  nh.param("sdk_mac_buffer_limit", mac_buffer_limit_, 360);
  ros::NodeHandle io;
  transport->batches = io.advertise<std_msgs::Float64MultiArray>("/jog_sdk/point_batches", 20);
  transport->feedback = io.subscribe<sensor_msgs::JointState>(
      "/jog_ros_control/raw_joint_states", 1,
      [this](const sensor_msgs::JointState::ConstPtr& message) {
        if (message->position.size() != kDof || message->header.stamp.isZero()) return;
        JointPoint point;
        std::copy(message->position.begin(), message->position.end(), point.begin());
        if (!finitePoint(point)) return;
        const double age = std::max(0.0, (ros::Time::now() - message->header.stamp).toSec());
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        pushed_position_ = point;
        feedback_received_ = Clock::now() - std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(age));
        have_pushed_position_ = true;
      }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  // Production connect() confirms a pushed sample before it starts feeding.
  // This local wait has its own callback queue and does not require the node's
  // main spinner (which starts after construction) to be running yet.
  const auto first = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/jog_ros_control/raw_joint_states", io, ros::Duration(3.0));
  if (!first || first->position.size() != kDof) return false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    std::copy(first->position.begin(), first->position.end(), pushed_position_.begin());
    feedback_received_ = Clock::now();
    have_pushed_position_ = true;
  }
  flow_.reset(new SdkFlowControl(mac_buffer_limit_ / 6, 0.15, Clock::now(),
                                SdkFlowControl::Policy::SequentialMac));
  connected_.store(true);
  running_.store(true);
  output_thread_ = std::thread(&DirectSdkBackend::outputLoop, this);
  ROS_INFO("[fake_sdk] Test transport only: no SDK login, startup or motion connection");
  return true;
}

bool DirectSdkBackend::readState(JointPoint& point)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!have_pushed_position_ || std::chrono::duration<double>(Clock::now() - feedback_received_).count() > feedback_timeout_)
    return false;
  point = pushed_position_;
  return true;
}

bool DirectSdkBackend::healthy() { return connected_.load() && running_.load(); }

std::string DirectSdkBackend::failureReason() const
{
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return failure_reason_;
}

void DirectSdkBackend::shutdown()
{
  running_.store(false);
  if (output_thread_.joinable()) output_thread_.join();
  connected_.store(false);
  transport.reset();
}

void DirectSdkBackend::outputLoop()
{
  const SdkJogTiming timing(mac_buffer_target_);
  SdkPointPacer pacer(timing.target_points);
  std::deque<Clock::time_point> execution;
  std::deque<std::pair<Clock::time_point, Clock::time_point>> accepted_points;
  bool started = false;
  unsigned batches = 0;
  Clock::time_point first_send{};
  auto fail = [&](const char* reason) {
    ROS_ERROR("[fake_sdk] %s", reason);
    std::lock_guard<std::mutex> lock(failure_mutex_);
    failure_reason_ = reason;
    running_.store(false);
  };
  while (running_.load() && ros::ok())
  {
    if (!have_last_position_)
    {
      if (!readState(last_position_))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      have_last_position_ = true;
    }
    const auto query_started = Clock::now();
    while (!execution.empty() && execution.front() <= query_started) execution.pop_front();
    // Earliest snapshot, before a slow reply. No consumption is invented from
    // SDK return time; the simulated cabinet executes accepted points at 5ms.
    const auto sampled = query_started - std::chrono::milliseconds(transport->cache_lag_ms);
    while (!accepted_points.empty() && accepted_points.front().second < sampled)
      accepted_points.pop_front();
    auto observed = static_cast<std::size_t>(std::count_if(accepted_points.begin(), accepted_points.end(),
        [&](const std::pair<Clock::time_point,Clock::time_point>& point) {
          return point.first <= sampled && point.second > sampled;
        }));
    const auto token = flow_->beginQuery(query_started);
    if (!token) { fail("simulated diagnosis start failed"); break; }
    MotionTimingTrace::Event query_event;
    if (timing_trace_)
    {
      query_event.kind = MotionTimingTrace::QueryBegin;
      query_event.query_id = ++trace_query_id_;
      query_event.begin_ns = MotionTimingTrace::ns(query_started);
      timing_trace_->append(query_event);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(transport->diagnosis_delay_ms));
    if (timing_trace_)
    {
      query_event.kind = MotionTimingTrace::QueryEnd;
      query_event.end_ns = MotionTimingTrace::ns();
      query_event.mac_scalars = observed * kDof;
      timing_trace_->append(query_event);
    }
    if (!flow_->acceptReply(token, observed * kDof, Clock::now()))
    { fail("simulated MAC admission failed"); break; }
    if (started && observed > 0) mac_confirmed_.store(true);
    if (started && !mac_confirmed_.load())
    {
      if (Clock::now() - first_send >= std::chrono::milliseconds(300))
      { fail("simulated first MAC confirmation timeout"); break; }
      continue;  // Cached startup zero grants no permission to re-send.
    }
    if (started && observed == 0) { fail("simulated MAC underflow"); break; }
    const auto deficit = timing.wanted(observed * kDof);
    if (started && !pacer.started()) pacer.start(Clock::now(), deficit);
    if (started && deficit == 0) pacer.discard(Clock::now());
    const auto wanted = started ? pacer.allowance(deficit, Clock::now()) : deficit;
    if (wanted == 0) continue;
    const auto batch_id = timing_trace_ ? ++trace_batch_id_ : 0;
    std_msgs::Float64MultiArray batch;
    for (std::size_t index = 0; index < wanted; ++index)
    {
      JointPoint point = last_position_;
      PointTiming point_timing;
      if (!point_generator_(point, timing_trace_ ? &point_timing : nullptr)) break;
      MotionTimingTrace::Event event;
      if (timing_trace_)
      {
        event.kind = MotionTimingTrace::Point;
        event.id = ++trace_point_id_;
        event.batch_id = batch_id;
        event.command_id = point_timing.command_id;
        event.target_id = point_timing.target_id;
        event.begin_ns = point_timing.begin_ns;
        event.locked_ns = point_timing.locked_ns;
        event.end_ns = point_timing.end_ns;
        event.q = point;
        event.velocity = point_timing.velocity;
        event.desired = point_timing.desired;
        timing_trace_->append(event);
      }
      if (!started && batch.data.empty() && point == last_position_) break;
      if (!finitePoint(point)) { fail("invalid simulated SDK point"); break; }
      batch.data.insert(batch.data.end(), point.begin(), point.end());
      last_position_ = point;
      if (timing_trace_)
      {
        event.kind = MotionTimingTrace::BatchAppend;
        event.begin_ns = MotionTimingTrace::ns();
        event.count = batch.data.size() / kDof;
        timing_trace_->append(event);
      }
    }
    if (batch.data.empty())
    {
      if (started) { fail("simulated generator stopped"); break; }
      continue;
    }
    if (batch.data.size() != wanted * kDof)
    {
      fail("incomplete simulated SDK batch");
      break;
    }
    if (!flow_->reserve(wanted, Clock::now()) || (started && !pacer.reserve(wanted, Clock::now())))
    { fail("simulated reserve exceeded capacity"); break; }
    auto accept = [&] {
      const auto accepted = Clock::now();
      auto tail = execution.empty() ? accepted : std::max(accepted, execution.back());
      for (std::size_t i = 0; i < wanted; ++i)
      {
        tail += std::chrono::milliseconds(5);
        execution.push_back(tail);
        accepted_points.push_back({accepted, tail});
      }
      const auto physical = std::count_if(execution.begin(), execution.end(),
          [&](Clock::time_point point) { return point > accepted; });
      if (physical > mac_buffer_limit_ / 6)
        fail("simulated physical MAC exceeded capacity with cached diagnosis");
      transport->batches.publish(batch);
    };
    if (!started) first_send = Clock::now();
    MotionTimingTrace::Event send_event;
    if (timing_trace_)
    {
      send_event.kind = MotionTimingTrace::SendBegin;
      send_event.id = batches + 1;
      send_event.batch_id = batch_id;
      send_event.query_id = trace_query_id_;
      send_event.begin_ns = MotionTimingTrace::ns();
      send_event.count = wanted;
      send_event.mac_scalars = observed * kDof;
      timing_trace_->append(send_event);
    }
    if (!transport->accept_late) accept();
    std::this_thread::sleep_for(std::chrono::milliseconds(batches == 0 ? 17 : transport->send_delay_ms));
    if (transport->accept_late) accept();
    if (timing_trace_)
    {
      send_event.kind = MotionTimingTrace::SendEnd;
      send_event.end_ns = MotionTimingTrace::ns();
      timing_trace_->append(send_event);
    }
    if (!flow_->completeSend(wanted))
    { fail("simulated acknowledgement failed"); break; }
    ++batches;
    started = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
