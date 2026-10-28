#include <aubo_ros_control/aubo_hardware_interface.h>
#include <aubo_ros_control/fixed_step_clock.h>
#include <gtest/gtest.h>
#include <controller_manager/controller_manager.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <future>
#include <fstream>
#include <cstdio>

#include <algorithm>
#include <limits>

namespace aubo_ros_control
{
struct AuboHardwareStreamingTestAccess
{
    static void prepare(AuboHardwareInterface& hw, ros::NodeHandle& nh)
    {
        // Never init(), login, startup or enter TCP2CANBUS on a real controller.
        hw.controller_connected_flag_.store(true);
        hw.feed_thread_running_.store(true);
        hw.in_tcp2canbus_mode_ = true;
        hw.position_controller_started_.store(true);
        hw.stream_reference_initialized_ = true;
        hw.flow_diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("flow", 10);
    }
    static void disarm(AuboHardwareInterface& hw)
    {
        hw.feed_thread_running_.store(false);
        hw.in_tcp2canbus_mode_ = false;
    }
    static void command(AuboHardwareInterface& hw, double position)
    {
        hw.joint_position_cmd_.fill(position);
        hw.write(ros::Time::now(), ros::Duration(0.005));
    }
    static std::vector<aubo_robot_namespace::wayPoint_S> pop(AuboHardwareInterface& hw, int count)
    { return hw.tryPopWaypoint(count); }
    static void feed(AuboHardwareInterface& hw) { hw.publishWaypointToRobot(); }
    static void finish(AuboHardwareInterface& hw) { hw.feed_thread_running_.store(false); }
    static double acceleration(AuboHardwareInterface& hw) { return hw.command_acceleration_max_.load(); }
    static std::size_t queued(AuboHardwareInterface& hw) { return hw.ros_motion_queue_.size_approx(); }
    static void resetStartup(AuboHardwareInterface& hw, double actual, double old_snapshot)
    {
        hw.position_controller_started_.store(false);
        hw.stream_reference_initialized_ = false;
        hw.startup_reference_seeded_.store(false);
        hw.joint_filter_.fill(old_snapshot);
        hw.joint_position_cmd_.fill(old_snapshot);
        for (auto& position : hw.joint_position_rt_) position.store(actual);
    }
    static bool seeded(AuboHardwareInterface& hw) { return hw.startup_reference_seeded_.load(); }
    static double jerk(AuboHardwareInterface& hw) { return hw.command_jerk_max_.load(); }
    static const std::vector<std::string>& names(AuboHardwareInterface& hw) { return hw.joint_names_; }
    static hardware_interface::ControllerInfo controller(AuboHardwareInterface& hw, bool position, bool full = true)
    {
        hardware_interface::ControllerInfo info;
        info.name = position ? "arm" : "states";
        std::set<std::string> resources(hw.joint_names_.begin(), hw.joint_names_.end());
        if (!full) resources.erase(hw.joint_names_.back());
        info.claimed_resources.emplace_back(position ? "hardware_interface::PositionJointInterface" :
                                                     "hardware_interface::JointStateInterface", resources);
        return info;
    }
    static void registerInterfaces(AuboHardwareInterface& hw)
    {
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            hardware_interface::JointStateHandle state(hw.joint_names_[i], &hw.joint_position_[i], &hw.joint_velocity_[i], &hw.joint_effort_[i]);
            hw.joint_state_interface_.registerHandle(state);
            hw.position_joint_interface_.registerHandle(hardware_interface::JointHandle(state, &hw.joint_position_cmd_[i]));
        }
        hw.registerInterface(&hw.joint_state_interface_);
        hw.registerInterface(&hw.position_joint_interface_);
    }
};
}  // namespace aubo_ros_control

using Access = aubo_ros_control::AuboHardwareStreamingTestAccess;
using aubo_ros_control::NUM_JOINTS;

class StreamHardware : public aubo_ros_control::AuboHardwareInterface
{
public:
    explicit StreamHardware(ros::NodeHandle& nh) : AuboHardwareInterface(nh) { Access::prepare(*this, nh); }
    ~StreamHardware() override { Access::disarm(*this); }
    int actual{0}, queries{0}, batches{0}, starvations{0}, max_actual{0};
    int first_batch{0}, sends_after_fault{0};
    bool inject_slow_query{false}, interrupt{false}, empty_after_confirmation{false};
    bool fail_send{false}, fail_diagnosis{false};
    bool inject_partial_watermark{false};
    bool cached_startup_zero{false};
    int max_requested_when_partial{0};
    int diagnosis_ms{20}, send_ms{20}, max_queries{70};
    std::size_t produced{0}, sent{0};
    std::size_t minimum_refill_batch{std::numeric_limits<std::size_t>::max()};

private:
    int now_ms_{0}, next_produced_ms_{5}, next_consumed_ms_{0};
    void advance(int ms)
    {
        const int end = now_ms_ + ms;
        while (next_produced_ms_ <= end)
        {
            // A fixed-step producer supplies all stationary samples even when
            // the sender was blocked. Production never grants cabinet capacity.
            Access::command(*this, 0.0);
            ++produced;
            next_produced_ms_ += 5;
        }
        while (batches > 0 && next_consumed_ms_ <= end)
        {
            if (actual == 0) ++starvations;
            else --actual;
            next_consumed_ms_ += 5;
        }
        now_ms_ = end;
    }
    int readMacDiagnosis(aubo_robot_namespace::RobotDiagnosis& diagnosis) override
    {
        ++queries;
        const int snapshot = actual;
        // Conservative mock: the reply describes request-start occupancy.
        advance(inject_slow_query && queries == 20 ? 167 : diagnosis_ms);
        diagnosis = {};
        diagnosis.macTargetPosDataSize = inject_partial_watermark && batches > 0 ? 295 : snapshot * 6;
        if (cached_startup_zero && batches > 0) diagnosis.macTargetPosDataSize = 0;
        if (interrupt && batches >= 3) diagnosis.macDataInterruptWarning = 1;
        if (empty_after_confirmation && batches >= 3) diagnosis.macTargetPosDataSize = 0;
        if (queries >= max_queries) Access::finish(*this);
        return fail_diagnosis && batches >= 3 ? -1 : aubo_robot_namespace::InterfaceCallSuccCode;
    }
    int sendWaypoints(const std::vector<aubo_robot_namespace::wayPoint_S>& points) override
    {
        if (!isConnected()) ++sends_after_fault;
        if (first_batch == 0) first_batch = static_cast<int>(points.size());
        if (inject_partial_watermark && batches > 0)
            max_requested_when_partial = std::max(max_requested_when_partial, static_cast<int>(points.size()));
        if (fail_send && batches >= 3) return -1;
        if (batches > 0) minimum_refill_batch = std::min(minimum_refill_batch, points.size());
        sent += points.size();
        if (batches == 0) next_consumed_ms_ = now_ms_ + 5;
        actual += static_cast<int>(points.size());
        ++batches;
        max_actual = std::max(max_actual, actual);
        advance(send_ms);
        return aubo_robot_namespace::InterfaceCallSuccCode;
    }
};

TEST(AuboHardwareStreaming, HoldsAndSubMicroradianSamplesRetainEveryFiveMilliseconds)
{
    ros::NodeHandle nh("/stream_samples");
    StreamHardware hw(nh);
    for (int i = 0; i < 60; ++i) Access::command(hw, 0.0);
    for (int i = 1; i <= 10; ++i) Access::command(hw, i * 1e-7);
    auto points = Access::pop(hw, 70);
    ASSERT_EQ(points.size(), 70u);
    for (int i = 0; i < 60; ++i) EXPECT_DOUBLE_EQ(points[i].jointpos[0], 0.0);
    for (int i = 1; i <= 10; ++i) EXPECT_DOUBLE_EQ(points[59 + i].jointpos[0], i * 1e-7);
    EXPECT_TRUE(hw.isConnected());
}

TEST(AuboHardwareStreaming, SignedReversalTriggersAccelerationProtection)
{
    ros::NodeHandle nh("/stream_reversal");
    StreamHardware hw(nh);
    Access::command(hw, 0.00025);  // +0.05 rad/s, +10 rad/s^2
    ASSERT_EQ(Access::pop(hw, 1).size(), 1u);
    Access::command(hw, 0.0);      // -0.05 rad/s, -20 rad/s^2 (old abs(v) check missed it)
    EXPECT_TRUE(Access::pop(hw, 1).empty());
    EXPECT_FALSE(hw.isConnected());
    EXPECT_NEAR(Access::acceleration(hw), 20.0, 1e-9);
}

TEST(AuboHardwareStreaming, OverspeedRejectsWholeBatchWithoutExpandingOrSendingIt)
{
    ros::NodeHandle nh("/stream_overspeed");
    StreamHardware hw(nh);
    Access::command(hw, 0.0);
    Access::command(hw, 0.1);
    EXPECT_TRUE(Access::pop(hw, 2).empty());
    EXPECT_FALSE(hw.isConnected());
    EXPECT_EQ(hw.batches, 0);
}

TEST(AuboHardwareStreaming, QueueOverflowLatchesRatherThanDroppingAndContinuing)
{
    ros::NodeHandle nh("/stream_overflow");
    StreamHardware hw(nh);
    for (int i = 0; i < 20000 && hw.isConnected(); ++i) Access::command(hw, 0.0);
    EXPECT_FALSE(hw.isConnected());
}

TEST(AuboHardwareStreaming, NonfiniteCommandLatchesRatherThanSkippingOneSample)
{
    ros::NodeHandle nh("/stream_nonfinite");
    StreamHardware hw(nh);
    Access::command(hw, std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(hw.isConnected());
    EXPECT_TRUE(Access::pop(hw, 1).empty());
}

TEST(AuboHardwareStreaming, FullPreloadSurvivesIsolated167msDiagnosisAndIdleHolds)
{
    ros::NodeHandle nh("/stream_slow_query");
    StreamHardware hw(nh);
    hw.inject_slow_query = true;
    Access::feed(hw);
    EXPECT_EQ(hw.first_batch, 50);
    EXPECT_GT(hw.batches, 20);
    EXPECT_EQ(hw.starvations, 0);
    EXPECT_LE(hw.max_actual, 50);
    EXPECT_TRUE(hw.isConnected());
}

TEST(AuboHardwareStreaming, PartialPointDoesNotRoundAvailableSpaceUp)
{
    ros::NodeHandle nh("/stream_partial");
    StreamHardware hw(nh);
    hw.inject_partial_watermark = true;
    Access::feed(hw);
    EXPECT_EQ(hw.batches, 1);  // 295 scalars leaves less than one whole point below target 300.
    EXPECT_EQ(hw.max_requested_when_partial, 0);
}

TEST(AuboHardwareStreaming, CachedStartupZeroNeverAuthorizesRepeatedFullPreloads)
{
    ros::NodeHandle nh("/stream_startup_cache");
    StreamHardware hw(nh);
    hw.cached_startup_zero = true;
    Access::feed(hw);
    EXPECT_EQ(hw.first_batch, 50);
    EXPECT_EQ(hw.batches, 1);
    EXPECT_LE(hw.max_actual, 50);
}

TEST(AuboHardwareStreaming, CabinetInterruptAndConfirmedEmptyNeverResumeStaleMotion)
{
    for (bool warning : {false, true})
    {
        ros::NodeHandle nh(warning ? "/stream_interrupt" : "/stream_empty");
        StreamHardware hw(nh);
        hw.interrupt = warning;
        hw.empty_after_confirmation = !warning;
        Access::feed(hw);
        EXPECT_FALSE(hw.isConnected());
        EXPECT_EQ(hw.batches, 3);
        EXPECT_EQ(hw.sends_after_fault, 0);
    }
}

TEST(AuboHardwareStreaming, DiagnosisAndSendFailuresStopOutput)
{
    for (bool diagnosis : {false, true})
    {
        ros::NodeHandle nh(diagnosis ? "/stream_query_failure" : "/stream_send_failure");
        StreamHardware hw(nh);
        hw.fail_diagnosis = diagnosis;
        hw.fail_send = !diagnosis;
        Access::feed(hw);
        EXPECT_FALSE(hw.isConnected());
        EXPECT_EQ(hw.batches, 3);
        EXPECT_EQ(hw.sends_after_fault, 0);
    }
}

TEST(AuboHardwareStreaming, LateControllerWakeupReplaysIntermediatePositions)
{
    using Clock = aubo_ros_control::FixedStepClock;
    const Clock::Time start{};
    Clock clock(start);
    ros::NodeHandle nh("/stream_late_controller");
    StreamHardware hw(nh);
    int sample = 0;
    // A smooth q(t)=0.1*t^2 path, including a 50ms host scheduling pause.
    for (int wake_ms : {5, 10, 15, 20, 70, 75, 80, 85, 90, 95, 100})
    {
        auto due = clock.pending(start + std::chrono::milliseconds(wake_ms));
        for (std::size_t i = 0; i < due; ++i)
        {
            const double t = ++sample * 0.005;
            Access::command(hw, 0.1 * t * t);
            clock.advance();
        }
    }
    const auto points = Access::pop(hw, 100);
    ASSERT_EQ(points.size(), 20u);
    EXPECT_TRUE(hw.isConnected());
    EXPECT_NEAR(Access::acceleration(hw), 0.2, 1e-9);
    // Old behavior jumped from q(.020) to q(.070) in a SINGLE 5ms slot.
    const double old_velocity = (0.1 * 0.07 * 0.07 - 0.1 * 0.02 * 0.02) / 0.005;
    EXPECT_GT(old_velocity, 4.0 * 0.1 * 2.0 * 0.1);
    EXPECT_EQ(clock.pending(start + std::chrono::milliseconds(100)), 0u);
    EXPECT_EQ(clock.pending(start + std::chrono::milliseconds(500)), 80u);
}

TEST(AuboHardwareStreaming, FastLinkBatchesFourSamplesWithoutDroppingOrStarving)
{
    ros::NodeHandle nh("/stream_small_batch");
    StreamHardware hw(nh);
    hw.diagnosis_ms = 5; hw.send_ms = 1; hw.max_queries = 160;
    Access::feed(hw);
    EXPECT_TRUE(hw.isConnected());
    EXPECT_EQ(hw.first_batch, 50);
    EXPECT_GT(hw.batches, 10);
    EXPECT_GE(hw.minimum_refill_batch, 4u);
    EXPECT_EQ(hw.produced, hw.sent + Access::queued(hw));
    EXPECT_EQ(hw.starvations, 0);
    EXPECT_LE(hw.max_actual, 50);
}

TEST(AuboHardwareStreaming, StartupWaitsForAllPositionClaimsAndSeedsFromFreshControllerHold)
{
    ros::NodeHandle nh("/stream_startup_seed"); StreamHardware hw(nh);
    Access::resetStartup(hw, .4, .4000036);
    Access::command(hw, .4000036);
    EXPECT_EQ(Access::queued(hw), 0u);
    hw.doSwitch({Access::controller(hw, false)}, {});
    Access::command(hw, .4000036);
    EXPECT_EQ(Access::queued(hw), 0u);
    hw.doSwitch({Access::controller(hw, true, false)}, {});
    Access::command(hw, .4000036);
    EXPECT_EQ(Access::queued(hw), 0u);
    hw.doSwitch({Access::controller(hw, true)}, {});
    for (int i = 0; i < 50; ++i) Access::command(hw, .4);
    auto points = Access::pop(hw, 50);
    ASSERT_EQ(points.size(), 50u);
    EXPECT_TRUE(Access::seeded(hw));
    EXPECT_DOUBLE_EQ(Access::acceleration(hw), 0.0);
    EXPECT_DOUBLE_EQ(Access::jerk(hw), 0.0);
    for (const auto& point : points) EXPECT_DOUBLE_EQ(point.jointpos[0], .4);
}

TEST(AuboHardwareStreaming, StartupRejectsLargeCommandFeedbackMismatch)
{
    for (const bool invalid_feedback : {false,true})
    {
        ros::NodeHandle nh("/stream_bad_seed"); StreamHardware hw(nh);
        Access::resetStartup(hw, invalid_feedback ? std::numeric_limits<double>::quiet_NaN() : .4, .4);
        hw.doSwitch({Access::controller(hw, true)}, {});
        Access::command(hw, invalid_feedback ? .4 : .42);
        EXPECT_TRUE(Access::pop(hw, 1).empty());
        EXPECT_FALSE(hw.isConnected());
        EXPECT_FALSE(Access::seeded(hw));
    }
}

TEST(AuboHardwareStreaming, RealControllerManagerStartupProducesNoStaleHoldPrefix)
{
    ros::NodeHandle nh("/stream_real_controller_start"); StreamHardware hw(nh);
    Access::resetStartup(hw, .4, .4000036); Access::registerInterfaces(hw);
    std::string urdf = "<robot name='start_test'><link name='link0'/>";
    const auto names = Access::names(hw);
    for (std::size_t i = 0; i < names.size(); ++i)
        urdf += "<link name='link" + std::to_string(i+1) + "'/><joint name='" + names[i] +
                "' type='revolute'><parent link='link" + std::to_string(i) + "'/><child link='link" + std::to_string(i+1) +
                "'/><axis xyz='0 0 1'/><limit lower='-3' upper='3' effort='100' velocity='0.5'/></joint>";
    urdf += "</robot>";
    nh.setParam("robot_description", urdf); nh.setParam("arm/robot_description", urdf);
    nh.setParam("arm/type", "position_controllers/JointTrajectoryController"); nh.setParam("arm/joints", names);
    controller_manager::ControllerManager manager(&hw, nh);
    ASSERT_TRUE(manager.loadController("arm"));
    const ros::Duration dt(.005);
    for (int i = 0; i < 5; ++i) { const auto now = ros::Time::now(); hw.read(now, dt); manager.update(now, dt); hw.write(now, dt); }
    EXPECT_EQ(Access::queued(hw), 0u);
    auto switched = std::async(std::launch::async, [&] { return manager.switchController({"arm"}, {}, 2, false, 1.0); });
    for (int i = 0; i < 100; ++i)
    {
        const auto now = ros::Time::now(); hw.read(now, dt); manager.update(now, dt); hw.write(now, dt);
        ros::WallDuration(.005).sleep();
        if (Access::queued(hw) >= 20 && switched.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) break;
    }
    ASSERT_EQ(switched.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    ASSERT_TRUE(switched.get());
    auto points = Access::pop(hw, 100);
    ASSERT_GE(points.size(), 20u);
    for (const auto& point : points) EXPECT_DOUBLE_EQ(point.jointpos[0], .4);
    EXPECT_DOUBLE_EQ(Access::acceleration(hw), 0.0); EXPECT_DOUBLE_EQ(Access::jerk(hw), 0.0);
    // Exercise the actual JTC command callback too: its first future P/V/A
    // point must bridge a 10-microradian feedback offset over 0.2s, not 5ms.
    ros::AsyncSpinner spinner(1); spinner.start();
    auto command = nh.advertise<trajectory_msgs::JointTrajectory>("arm/command", 1);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (command.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) ros::WallDuration(.005).sleep();
    ASSERT_GT(command.getNumSubscribers(), 0u);
    trajectory_msgs::JointTrajectory message; message.joint_names = names;
    trajectory_msgs::JointTrajectoryPoint start, end;
    start.positions.assign(NUM_JOINTS,.40001); start.velocities.assign(NUM_JOINTS,0.0); start.accelerations.assign(NUM_JOINTS,0.0);
    start.time_from_start = ros::Duration(.2);
    end = start; end.positions.assign(NUM_JOINTS,.401); end.time_from_start = ros::Duration(.7);
    message.points = {start,end}; command.publish(message);
    for (int i = 0; i < 200; ++i)
    {
        const auto now = ros::Time::now(); hw.read(now,dt); manager.update(now,dt); hw.write(now,dt); ros::WallDuration(.005).sleep();
    }
    points = Access::pop(hw,500);
    ASSERT_FALSE(points.empty()); EXPECT_TRUE(hw.isConnected());
    EXPECT_NEAR(points.back().jointpos[0],.401,1e-9);
    EXPECT_LT(Access::acceleration(hw),.04); EXPECT_LT(Access::jerk(hw),1.0);
}

TEST(AuboHardwareStreaming, HardwareTraceIsBoundedAndPreservesExistingCaptures)
{
    char directory[] = "/tmp/aubo_hardware_trace_test_XXXXXX";
    ASSERT_NE(mkdtemp(directory), nullptr);
    const std::string prefix = std::string(directory) + "/capture";
    {
        aubo_ros_control::HardwareMotionTrace trace(prefix, 8);
        std::thread commands([&] {
            for (std::uint64_t i = 1; i <= 16; ++i) {
                aubo_ros_control::HardwareMotionTrace::Command e;
                e.id=i; e.seed=i==1; e.sample_ns=i*5000000; e.q.fill(.4); trace.append(e);
            }
        });
        std::thread calls([&] {
            for (std::uint64_t i = 1; i <= 70; ++i) {
                aubo_ros_control::HardwareMotionTrace::Rpc e;
                e.id=i; e.begin_ns=100; e.end_ns=200; e.port_hint=12345;
                e.before.valid=e.after.valid=true; e.before.detail_count=e.after.detail_count=1;
                e.before.sockets[0].local_port=e.after.sockets[0].local_port=12345;
                e.after.sockets[0].bytes_sent=100+i;
                trace.append(e);
            }
        });
        commands.join(); calls.join(); ASSERT_TRUE(trace.dumpAfterJoin());
        std::ifstream points(prefix+".commands.csv"), rpc(prefix+".rpc.csv");
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(points,line))); EXPECT_NE(line.find("dropped=8"),std::string::npos);
        ASSERT_TRUE(static_cast<bool>(std::getline(rpc,line))); EXPECT_NE(line.find("dropped=6"),std::string::npos);
        std::size_t point_lines=0,rpc_lines=0;
        while(std::getline(points,line)) ++point_lines;
        while(std::getline(rpc,line)) ++rpc_lines;
        EXPECT_EQ(point_lines,9u); EXPECT_EQ(rpc_lines,129u);
        EXPECT_THROW(aubo_ros_control::HardwareMotionTrace(prefix,8),std::runtime_error);
        std::ifstream preserved(prefix+".commands.csv");
        std::getline(preserved,line); EXPECT_NE(line.find("hardware_commands_v1"),std::string::npos);
    }
    const auto blocked_prefix = std::string(directory) + "/blocked";
    {
        aubo_ros_control::HardwareMotionTrace trace(blocked_prefix,8);
        std::ofstream prior_export(blocked_prefix+".rpc.csv.export"); prior_export << "unfinished"; prior_export.close();
        EXPECT_FALSE(trace.dumpAfterJoin());
        std::ifstream placeholder(blocked_prefix+".commands.csv"); std::string line; std::getline(placeholder,line);
        EXPECT_NE(line.find("incomplete"),std::string::npos);
        std::ifstream preserved(blocked_prefix+".rpc.csv.export");std::getline(preserved,line);EXPECT_EQ(line,"unfinished");
    }
    for (const auto& capture : {prefix,blocked_prefix})
        for (const auto* suffix : {".commands.csv",".rpc.csv",".commands.csv.export",".rpc.csv.export"})
            std::remove((capture+suffix).c_str());
    rmdir(directory);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aubo_hardware_streaming_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
