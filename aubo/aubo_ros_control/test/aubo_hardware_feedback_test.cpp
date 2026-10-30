#include <aubo_ros_control/aubo_hardware_interface.h>
#include <gtest/gtest.h>
#include <sensor_msgs/JointState.h>

#include <chrono>
#include <algorithm>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

namespace aubo_ros_control
{
struct AuboHardwareFeedbackTestAccess
{
    static void prepare(AuboHardwareInterface& hardware, ros::NodeHandle& nh)
    {
        // No init(), SDK login, startup, stream mode or feed thread in this test.
        hardware.use_realtime_feedback_ = true;
        hardware.enable_motion_control_ = true;
        hardware.real_robot_exist_ = true;
        hardware.controller_connected_flag_.store(true);
        hardware.accept_feedback_callbacks_.store(true);
        hardware.raw_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("raw", 1);
    }
    static void receive(AuboHardwareInterface& hardware, const aubo_robot_namespace::wayPoint_S* point)
    { hardware.receiveWaypoint(point, &hardware); }
    static void diagnose(AuboHardwareInterface& hardware)
    { hardware.timerCallback(ros::TimerEvent{}); }
    static std::array<double, NUM_JOINTS> position(AuboHardwareInterface& hardware)
    {
        hardware.read(ros::Time::now(), ros::Duration(0.005));
        return hardware.joint_position_;
    }
    static ros::WallTime sampledAt(AuboHardwareInterface& hardware)
    {
        std::lock_guard<std::mutex> lock(hardware.feedback_mutex_);
        return hardware.last_state_wall_time_;
    }
    static void stopCallbacks(AuboHardwareInterface& hardware)
    { hardware.accept_feedback_callbacks_.store(false); }
    static void startPolling(AuboHardwareInterface& hardware)
    {
        hardware.feedback_poll_running_.store(true);
        hardware.feedback_poll_thread_ = std::thread(&AuboHardwareInterface::pollFeedback, &hardware);
    }
    static void stopPolling(AuboHardwareInterface& hardware)
    {
        hardware.feedback_poll_running_.store(false);
        hardware.feedback_ready_.notify_all();
        if (hardware.feedback_poll_thread_.joinable()) hardware.feedback_poll_thread_.join();
    }
    static double age(AuboHardwareInterface& hardware)
    {
        std::lock_guard<std::mutex> lock(hardware.feedback_mutex_);
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - hardware.last_feedback_sample_).count();
    }
    static std::uint64_t ignored(AuboHardwareInterface& hardware)
    { return hardware.ignored_push_samples_.load(); }
    static bool fresh(AuboHardwareInterface& hardware) { return hardware.motionFeedbackFresh(); }
    static void capture(AuboHardwareInterface& hardware, const std::string& prefix)
    { hardware.motion_trace_.reset(new HardwareMotionTrace(prefix, 64)); }
    static bool dumpCapture(AuboHardwareInterface& hardware)
    {
        const bool result = hardware.motion_trace_->dumpAfterJoin();
        hardware.motion_trace_.reset();
        return result;
    }
    static bool startup(AuboHardwareInterface& hardware)
    {
        aubo_robot_namespace::ROBOT_SERVICE_STATE result{};
        return hardware.startRobotPreservingTool(result);
    }
};
}

class SnapshotHardware : public aubo_ros_control::AuboHardwareInterface
{
public:
    explicit SnapshotHardware(ros::NodeHandle& nh) : AuboHardwareInterface(nh) {}
    std::atomic<int> mode{0}, calls{0};
    std::atomic<bool> entered{false}, release{false};
private:
    int readFeedbackSnapshot(aubo_robot_namespace::wayPoint_S& point) override
    {
        const int selected = mode.load();
        ++calls;
        if (selected == 1)
        {
            entered.store(true);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!release.load() && std::chrono::steady_clock::now() < deadline)
                ros::WallDuration(0.005).sleep();
        }
        else
            ros::WallDuration(selected == 3 ? 0.22 : 0.02).sleep();
        if (selected == 2) return -1;
        point = {};
        point.jointpos[0] = 0.1;
        return aubo_robot_namespace::InterfaceCallSuccCode;
    }
};

class ToolHardware : public aubo_ros_control::AuboHardwareInterface
{
public:
    explicit ToolHardware(ros::NodeHandle& nh) : AuboHardwareInterface(nh) {}
    aubo_robot_namespace::ToolDynamicsParam configured{}, received{};
    int read_result = 0, startup_result = 0, startup_calls = 0;
private:
    int readToolDynamics(aubo_robot_namespace::ToolDynamicsParam& tool) override
    { tool = configured; return read_result; }
    int startupRobot(const aubo_robot_namespace::ToolDynamicsParam& tool,
                     aubo_robot_namespace::ROBOT_SERVICE_STATE&) override
    { received = tool; ++startup_calls; return startup_result; }
};

TEST(AuboHardwareStartup, PreservesConfiguredPayloadAndCenterOfMass)
{
    ros::NodeHandle nh("/hardware_tool_test");
    ToolHardware hardware(nh);
    hardware.configured.payload = .75;
    hardware.configured.positionX = .01;
    hardware.configured.positionY = -.02;
    hardware.configured.positionZ = .08;
    ASSERT_TRUE(aubo_ros_control::AuboHardwareFeedbackTestAccess::startup(hardware));
    EXPECT_EQ(hardware.startup_calls, 1);
    EXPECT_DOUBLE_EQ(hardware.received.payload, .75);
    EXPECT_DOUBLE_EQ(hardware.received.positionX, .01);
    EXPECT_DOUBLE_EQ(hardware.received.positionY, -.02);
    EXPECT_DOUBLE_EQ(hardware.received.positionZ, .08);
    hardware.configured = {}; // A genuinely configured bare flange remains valid.
    EXPECT_TRUE(aubo_ros_control::AuboHardwareFeedbackTestAccess::startup(hardware));
    EXPECT_DOUBLE_EQ(hardware.received.payload, 0);
}

TEST(AuboHardwareStartup, ReadFailureInvalidToolAndStartupFailureCannotSucceed)
{
    using Access = aubo_ros_control::AuboHardwareFeedbackTestAccess;
    ros::NodeHandle nh("/hardware_tool_failure_test");
    ToolHardware hardware(nh);
    hardware.read_result = -1;
    EXPECT_FALSE(Access::startup(hardware));
    hardware.read_result = 0;
    hardware.configured.positionZ = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(Access::startup(hardware));
    hardware.configured = {};
    hardware.configured.payload = -1;
    EXPECT_FALSE(Access::startup(hardware));
    EXPECT_EQ(hardware.startup_calls, 0);
    hardware.configured = {};
    hardware.startup_result = -1;
    EXPECT_FALSE(Access::startup(hardware));
    EXPECT_EQ(hardware.startup_calls, 1);
}

TEST(AuboHardwareFeedback, QueryTakeoverRejectsDelayedPushesEvenWhileNextQueryIsBlocked)
{
    using Access = aubo_ros_control::AuboHardwareFeedbackTestAccess;
    ros::NodeHandle nh("/hardware_feedback_fallback_test");
    std::mutex mutex;
    std::vector<sensor_msgs::JointState> samples;
    auto subscriber = nh.subscribe<sensor_msgs::JointState>("raw", 50,
        [&](const sensor_msgs::JointState::ConstPtr& sample) {
            std::lock_guard<std::mutex> lock(mutex);
            samples.push_back(*sample);
        });
    SnapshotHardware hardware(nh);
    Access::prepare(hardware, nh);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (subscriber.getNumPublishers() == 0 && ros::WallTime::now() < deadline)
        ros::WallDuration(0.01).sleep();
    ASSERT_EQ(subscriber.getNumPublishers(), 1u);
    aubo_robot_namespace::wayPoint_S point{};
    Access::receive(hardware, &point);
    Access::startPolling(hardware);
    double max_age = 0.0;
    // No pushed frames during this interval: only actual snapshot RPCs can
    // provide fresh samples. The mock response itself takes 20 ms.
    for (int tick = 0; tick != 100; ++tick)
    {
        ros::WallDuration(0.005).sleep();
        max_age = std::max(max_age, Access::age(hardware));
    }
    EXPECT_GE(hardware.calls.load(), 5);
    EXPECT_LT(max_age, 0.15);
    {
        std::lock_guard<std::mutex> lock(mutex);
        EXPECT_GE(samples.size(), 5u);
    }
    // The query source has taken over. Simulate the recorded SDK burst while
    // the next query is blocked: callback receipt stamps MUST NOT grant those
    // older cabinet frames authority or refresh the feedback watchdog.
    hardware.mode.store(1);
    const auto query_deadline = ros::WallTime::now() + ros::WallDuration(1.0);
    while (!hardware.entered.load() && ros::WallTime::now() < query_deadline)
        ros::WallDuration(0.005).sleep();
    EXPECT_TRUE(hardware.entered.load());
    point.jointpos[0] = 0.5;
    for (int frame = 0; frame != 12; ++frame)
    {
        Access::receive(hardware, &point);
        ros::WallDuration(0.02).sleep();
    }
    EXPECT_GE(Access::ignored(hardware), 12u);
    EXPECT_GT(Access::age(hardware), 0.2);
    EXPECT_DOUBLE_EQ(Access::position(hardware)[0], 0.1);
    hardware.mode.store(2);
    hardware.release.store(true);
    ros::WallDuration(0.03).sleep();
    // A slow query keeps its request stamp; its return cannot freshen old data.
    EXPECT_DOUBLE_EQ(Access::position(hardware)[0], 0.1);
    // Failed queries must leave age increasing past the watchdog threshold.
    ros::WallDuration(0.2).sleep();
    EXPECT_GT(Access::age(hardware), 0.15);
    Access::stopPolling(hardware);

    // A successful but late reply retains its old REQUEST stamp. It must not
    // appear newly sampled simply because the RPC has finally returned.
    hardware.mode.store(3);
    const int before = hardware.calls.load();
    Access::startPolling(hardware);
    while (hardware.calls.load() == before) ros::WallDuration(0.005).sleep();
    Access::stopPolling(hardware);  // Wait for that single in-flight slow reply.
    EXPECT_GT(Access::age(hardware), 0.2);
    ros::WallDuration(0.03).sleep();
    {
        std::lock_guard<std::mutex> lock(mutex);
        ASSERT_FALSE(samples.empty());
        EXPECT_GT((ros::Time::now() - samples.back().header.stamp).toSec(), 0.2);
    }
    EXPECT_FALSE(Access::fresh(hardware));
    EXPECT_FALSE(hardware.isConnected());
}

TEST(AuboHardwareFeedback, FailedFirstFallbackCannotBeOverriddenByFreshlyStampedOldPushes)
{
    using Access = aubo_ros_control::AuboHardwareFeedbackTestAccess;
    ros::NodeHandle nh("/hardware_failed_takeover_test");
    SnapshotHardware hardware(nh);
    Access::prepare(hardware, nh);
    aubo_robot_namespace::wayPoint_S point{};
    point.jointpos[0] = -0.16720483;
    Access::receive(hardware, &point);
    hardware.mode.store(2);
    Access::startPolling(hardware);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(1.0);
    while (hardware.calls.load() == 0 && ros::WallTime::now() < deadline)
        ros::WallDuration(.005).sleep();
    ASSERT_GT(hardware.calls.load(), 0);
    // Seven recorded old positions were delivered within 0.306 ms. Their host
    // callback stamps increase, but no value may replace the accepted source.
    for (double q : {-0.13400634,-0.13924804,-0.14447445,-0.14971308,
                     -0.15582432,-0.16020873,-0.16457172})
    {
        point.jointpos[0] = q;
        Access::receive(hardware, &point);
    }
    EXPECT_EQ(Access::ignored(hardware), 7u);
    EXPECT_DOUBLE_EQ(Access::position(hardware)[0], -0.16720483);
    ros::WallDuration(.2).sleep();
    Access::stopPolling(hardware);
    EXPECT_FALSE(Access::fresh(hardware));
    EXPECT_FALSE(hardware.isConnected());
}

TEST(AuboHardwareFeedback, SlowQueryIsCapturedEvenWhenItsFeedbackIsAlreadyStale)
{
    using Access = aubo_ros_control::AuboHardwareFeedbackTestAccess;
    char directory[] = "/tmp/aubo_feedback_trace_XXXXXX";
    ASSERT_NE(mkdtemp(directory), nullptr);
    const std::string prefix = std::string(directory) + "/capture";
    ros::NodeHandle nh("/hardware_feedback_trace_test");
    SnapshotHardware hardware(nh);
    Access::prepare(hardware, nh);
    Access::capture(hardware, prefix);
    hardware.mode.store(3); // 220 ms response, longer than the freshness limit.
    Access::startPolling(hardware);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(1.0);
    while (hardware.calls.load() == 0 && ros::WallTime::now() < deadline)
        ros::WallDuration(.005).sleep();
    Access::stopPolling(hardware); // Join the in-flight RPC before exporting.
    EXPECT_EQ(hardware.calls.load(), 1);
    EXPECT_GT(Access::age(hardware), .2);
    ASSERT_TRUE(Access::dumpCapture(hardware));
    std::ifstream capture(prefix + ".rpc.csv");
    std::string line;
    int records = 0;
    while (std::getline(capture, line))
        if (line.find("feedback_query,1,0,") == 0) ++records;
    EXPECT_EQ(records, 2); // Before and after, including unavailable TCP data.
    std::remove((prefix + ".rpc.csv").c_str());
    std::remove((prefix + ".commands.csv").c_str());
    rmdir(directory);
}

class BlockingDiagnosisHardware : public aubo_ros_control::AuboHardwareInterface
{
public:
    explicit BlockingDiagnosisHardware(ros::NodeHandle& nh) : AuboHardwareInterface(nh) {}
    bool waitUntilBlocked()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, std::chrono::seconds(2), [this] { return entered_; });
    }
    void release()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }
private:
    int readSafetyDiagnosis(aubo_robot_namespace::RobotDiagnosis& diagnosis) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        changed_.wait_for(lock, std::chrono::seconds(3), [this] { return released_; });
        diagnosis = {};
        return aubo_robot_namespace::InterfaceCallSuccCode;
    }
    std::mutex mutex_;
    std::condition_variable changed_;
    bool entered_{false}, released_{false};
};

TEST(AuboHardwareFeedback, PushPublishesWhileDiagnosisBlockedAndDoesNotRefreshOldSamples)
{
    using Access = aubo_ros_control::AuboHardwareFeedbackTestAccess;
    ros::NodeHandle nh("/hardware_feedback_test");
    std::mutex samples_mutex;
    std::condition_variable samples_changed;
    std::vector<sensor_msgs::JointState> samples;
    auto subscriber = nh.subscribe<sensor_msgs::JointState>("raw", 20,
        [&](const sensor_msgs::JointState::ConstPtr& message) {
            std::lock_guard<std::mutex> lock(samples_mutex);
            samples.push_back(*message);
            samples_changed.notify_all();
        });
    BlockingDiagnosisHardware hardware(nh);
    Access::prepare(hardware, nh);
    const auto connection_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (subscriber.getNumPublishers() == 0 && ros::WallTime::now() < connection_deadline)
        ros::WallDuration(0.01).sleep();
    ASSERT_EQ(subscriber.getNumPublishers(), 1u);

    std::thread diagnosis([&] { Access::diagnose(hardware); });
    const bool diagnosis_blocked = hardware.waitUntilBlocked();
    if (!diagnosis_blocked)
    {
        hardware.release();
        diagnosis.join();
        FAIL() << "Push-mode timer did not enter the isolated diagnosis RPC";
    }
    aubo_robot_namespace::wayPoint_S point{};
    for (int frame = 0; frame != 12; ++frame)
    {
        for (int joint = 0; joint != aubo_ros_control::NUM_JOINTS; ++joint)
            point.jointpos[joint] = 0.1 * joint + 0.001 * frame;
        Access::receive(hardware, &point);
        ros::WallDuration(0.02).sleep();
    }
    // The response is still held; actual callback data must already be visible.
    {
        std::unique_lock<std::mutex> lock(samples_mutex);
        samples_changed.wait_for(lock, std::chrono::seconds(1), [&] { return samples.size() >= 8; });
        EXPECT_GE(samples.size(), 8u);
        if (samples.size() >= 2)
            EXPECT_GT((samples.back().header.stamp - samples.front().header.stamp).toSec(), 0.15);
    }
    EXPECT_TRUE(hardware.isConnected());
    const auto position = Access::position(hardware);
    for (int joint = 0; joint != aubo_ros_control::NUM_JOINTS; ++joint)
        EXPECT_DOUBLE_EQ(position[joint], point.jointpos[joint]);
    hardware.release();
    diagnosis.join();

    const auto sampled = Access::sampledAt(hardware);
    std::size_t count;
    {
        std::lock_guard<std::mutex> lock(samples_mutex);
        ASSERT_FALSE(samples.empty());
        EXPECT_EQ(samples.back().name.size(), 6u);
        EXPECT_EQ(samples.back().name.front(), "shoulder_joint");
        EXPECT_EQ(samples.back().name.back(), "wrist3_joint");
        EXPECT_EQ(samples.back().position.size(), 6u);
        EXPECT_EQ(samples.back().velocity.size(), 6u);
        count = samples.size();
    }
    Access::diagnose(hardware);  // A completed diagnosis is not a new joint sample.
    Access::receive(hardware, nullptr);
    auto invalid = point;
    invalid.jointpos[0] = std::numeric_limits<double>::quiet_NaN();
    Access::receive(hardware, &invalid);
    EXPECT_EQ(Access::sampledAt(hardware), sampled);
    Access::stopCallbacks(hardware);
    point.jointpos[0] += 1.0;
    Access::receive(hardware, &point);
    ros::WallDuration(0.04).sleep();
    EXPECT_EQ(Access::sampledAt(hardware), sampled);
    {
        std::lock_guard<std::mutex> lock(samples_mutex);
        EXPECT_EQ(samples.size(), count);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aubo_hardware_feedback_test");
    testing::InitGoogleTest(&argc, argv);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    const int result = RUN_ALL_TESTS();
    spinner.stop();
    return result;
}
