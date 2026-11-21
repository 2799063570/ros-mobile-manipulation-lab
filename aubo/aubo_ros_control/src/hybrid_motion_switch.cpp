#include <aubo_ros_control/aubo_hardware_interface.h>
#include <controller_manager_msgs/SwitchController.h>
#include <urdf/model.h>

#include <algorithm>

namespace aubo_ros_control
{
namespace
{
using Clock = std::chrono::steady_clock;
using Mode = ControlModeState;
constexpr std::size_t SERVO_INPUT_CAPACITY = 32;
constexpr std::size_t SERVO_INPUT_PRELOAD = 6;
constexpr double SERVO_POINT_TIMEOUT = 0.15;
// A position-controller restart uses measured joint positions. Keep any
// difference from the last hold below the feeder's per-sample acceleration bound.
constexpr double HOLD_POSITION_TOLERANCE = 0.0002;
}

bool AuboHardwareInterface::loadServoPositionLimits()
{
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("[AuboHW/hybrid] Missing or invalid robot_description");
        return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const auto joint = model.getJoint(joint_names_[i]);
        if (!joint || !joint->limits || !std::isfinite(joint->limits->lower) ||
            !std::isfinite(joint->limits->upper) || joint->limits->lower >= joint->limits->upper)
        {
            ROS_ERROR("[AuboHW/hybrid] Missing finite position limits for %s", joint_names_[i].c_str());
            return false;
        }
        servo_lower_limits_[i] = joint->limits->lower;
        servo_upper_limits_[i] = joint->limits->upper;
    }
    return true;
}

void AuboHardwareInterface::setupHybridControl(ros::NodeHandle& controller_nh)
{
    if (!hybrid_control_enabled_)
        return;

    control_mode_.active_mode = Mode::ROS_CONTROL;
    control_mode_.requested_mode = Mode::ROS_CONTROL;
    control_mode_.phase = Mode::STARTING;
    control_mode_pub_ = controller_nh.advertise<Mode>("control_mode_state", 1, true);
    servo_point_sub_ = controller_nh.subscribe(
        "servo_joint_point", SERVO_INPUT_CAPACITY, &AuboHardwareInterface::acceptServoPoint,
        this, ros::TransportHints().tcpNoDelay());
    controller_switch_client_ = controller_nh.serviceClient<controller_manager_msgs::SwitchController>(
        "controller_manager/switch_controller");
    control_mode_service_ = controller_nh.advertiseService(
        "set_control_mode", &AuboHardwareInterface::requestControlMode, this);
    publishControlMode("ROS_CONTROL selected; waiting for the trajectory controller");
}

bool AuboHardwareInterface::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>&)
{
    if (!hybrid_control_enabled_)
        return true;
    for (const auto& controller : start_list)
        for (const auto& claim : controller.claimed_resources)
            if (claim.hardware_interface == "hardware_interface::PositionJointInterface" &&
                (controller.name != trajectory_controller_ || !allow_trajectory_controller_start_.load()))
                return false;
    return true;
}

bool AuboHardwareInterface::requestControlMode(SetControlMode::Request& request,
                                               SetControlMode::Response& response)
{
    std::lock_guard<std::mutex> lock(hybrid_mutex_);
    response.switch_id = control_mode_.switch_id;
    if (!isConnected() || control_mode_.phase == Mode::FAULT)
        response.message = "hardware unavailable or faulted";
    else if (request.mode != Mode::ROS_CONTROL && request.mode != Mode::VISUAL_SERVO &&
             request.mode != Mode::HOLD)
        response.message = "invalid control mode";
    else if (request.mode == control_mode_.requested_mode)
    {
        response.accepted = true;
        response.message = "mode already selected or pending; check control_mode_state";
    }
    else if (control_mode_.phase != Mode::ACTIVE)
        response.message = "another switch is in progress";
    else if (!startup_reference_seeded_.load() ||
             (control_mode_.active_mode == Mode::ROS_CONTROL && !jtc_running_.load()))
        response.message = "wait for hardware and the trajectory controller to become ready";
    else
    {
        response.accepted = true;
        response.switch_id = ++control_mode_.switch_id;
        response.message = "accepted; wait for ACTIVE with this switch_id";
        control_mode_.requested_mode = request.mode;
        control_mode_.phase = Mode::WAIT_STILL;
        switch_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(hybrid_switch_timeout_));
        publishControlMode("waiting for the old command source to settle");
    }
    return true;
}

void AuboHardwareInterface::publishControlMode(const char* message)
{
    control_mode_.message = message;
    control_mode_pub_.publish(control_mode_);
}

void AuboHardwareInterface::failControlMode(const char* message)
{
    allow_trajectory_controller_start_.store(false);
    control_mode_.active_mode = Mode::HOLD;
    control_mode_.servo_session = 0;
    control_mode_.phase = Mode::FAULT;
    servo_points_.clear();
    publishControlMode(message);
    ROS_ERROR("[AuboHW/hybrid] %s", message);
    requestExecutionStop();
}

bool AuboHardwareInterface::hybridFeedbackAt(const std::array<double, NUM_JOINTS>& position)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const double age = std::chrono::duration<double>(Clock::now() - last_feedback_sample_).count();
    if (!have_feedback_sample_ || age > std::min(0.08, feedback_timeout_s_))
        return false;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const double actual = joint_position_rt_[i].load(std::memory_order_relaxed);
        const double velocity = joint_velocity_rt_[i].load(std::memory_order_relaxed);
        if (!std::isfinite(actual) || !std::isfinite(velocity) ||
            std::fabs(actual - position[i]) > HOLD_POSITION_TOLERANCE || std::fabs(velocity) > 0.01)
            return false;
    }
    return true;
}

void AuboHardwareInterface::updateHybridControl()
{
    if (!hybrid_control_enabled_)
        return;
    std::lock_guard<std::mutex> lock(hybrid_mutex_);
    const auto now = Clock::now();
    if (!isConnected() && control_mode_.phase != Mode::FAULT)
        return failControlMode("hardware disconnected or motion faulted");

    if (control_mode_.phase == Mode::STARTING)
    {
        if (jtc_running_.load() && startup_reference_seeded_.load())
        {
            control_mode_.phase = Mode::ACTIVE;
            publishControlMode("ROS_CONTROL is active");
        }
        return;
    }
    if (control_mode_.active_mode == Mode::ROS_CONTROL && !jtc_running_.load())
        return failControlMode("trajectory controller stopped outside the mode switch");

    if (control_mode_.active_mode == Mode::VISUAL_SERVO &&
        (servo_input_fault_ || std::chrono::duration<double>(now - last_servo_point_).count() >
            (servo_stream_started_ ? SERVO_POINT_TIMEOUT : 1.0)))
        return failControlMode("visual servo point stream is missing, stale or discontinuous");
    if (control_mode_.phase == Mode::ACTIVE || control_mode_.phase == Mode::FAULT)
        return;
    if (now >= switch_deadline_)
        return failControlMode("mode switch timed out; motion stopped");

    if (control_mode_.phase == Mode::WAIT_STILL)
    {
        if (stationary_command_samples_ < 10 || !hybridFeedbackAt(last_cmd_sent_) ||
            (control_mode_.active_mode == Mode::ROS_CONTROL && jtc_goal_active_.load()) ||
            (control_mode_.active_mode == Mode::VISUAL_SERVO && !servo_settled_))
            return;
        std::copy(last_cmd_sent_.begin(), last_cmd_sent_.end(), control_mode_.hold_position.begin());
        allow_trajectory_controller_start_.store(false);
        control_mode_.active_mode = Mode::HOLD;
        control_mode_.servo_session = 0;
        servo_points_.clear();
        stable_since_ = {};
        control_mode_.phase = Mode::DRAINING;
        publishControlMode("both sources blocked; confirming the SDK hold fence");
        return;
    }

    const bool start_jtc = control_mode_.requested_mode == Mode::ROS_CONTROL;
    if (control_mode_.phase == Mode::DRAINING)
    {
        if (confirmed_fence_.load() != control_mode_.switch_id || !hybridFeedbackAt(last_cmd_sent_))
        {
            stable_since_ = {};
            return;
        }
        if (stable_since_ == Clock::time_point{})
            stable_since_ = now;
        if (now - stable_since_ < std::chrono::milliseconds(100))
            return;
        if (start_jtc != jtc_running_.load())
        {
            allow_trajectory_controller_start_.store(start_jtc);
            control_mode_.phase = Mode::SWITCHING_CONTROLLER;
            publishControlMode(start_jtc ? "starting trajectory controller" : "stopping trajectory controller");
            controller_switch_result_ = std::async(std::launch::async,
                [client = controller_switch_client_, controller = trajectory_controller_, start_jtc]() mutable
                {
                    controller_manager_msgs::SwitchController call;
                    auto& list = start_jtc ? call.request.start_controllers : call.request.stop_controllers;
                    list.push_back(controller);
                    call.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
                    call.request.timeout = 1.0;
                    return client.call(call) && call.response.ok;
                });
            return;
        }
    }
    else
    {
        if (controller_switch_result_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return;
        if (!controller_switch_result_.get() || jtc_running_.load() != start_jtc ||
            !hybridFeedbackAt(last_cmd_sent_))
            return failControlMode("trajectory controller switch or feedback confirmation failed");
    }

    control_mode_.active_mode = control_mode_.requested_mode;
    control_mode_.phase = Mode::ACTIVE;
    if (control_mode_.active_mode == Mode::VISUAL_SERVO)
    {
        control_mode_.servo_session = ros::WallTime::now().toNSec();
        next_servo_sequence_ = 1;
        last_servo_point_ = now;
        servo_stream_started_ = servo_settled_ = servo_input_fault_ = false;
    }
    publishControlMode("requested control mode is active");
}

void AuboHardwareInterface::acceptServoPoint(const ServoJointPoint::ConstPtr& point)
{
    std::lock_guard<std::mutex> lock(hybrid_mutex_);
    if (control_mode_.active_mode != Mode::VISUAL_SERVO ||
        point->session != control_mode_.servo_session || servo_input_fault_)
        return;
    const double age = (ros::Time::now() - point->stamp).toSec();
    bool valid = !point->stamp.isZero() && age >= 0.0 && age <= SERVO_POINT_TIMEOUT &&
        point->sequence == next_servo_sequence_ && servo_points_.size() < SERVO_INPUT_CAPACITY;
    for (int i = 0; i < NUM_JOINTS; ++i)
        valid &= std::isfinite(point->position[i]) && point->position[i] >= servo_lower_limits_[i] &&
                 point->position[i] <= servo_upper_limits_[i];
    if (!valid)
    {
        servo_input_fault_ = true;
        return;
    }
    ++next_servo_sequence_;
    last_servo_point_ = Clock::now();
    servo_points_.push_back(*point);
}

bool AuboHardwareInterface::hybridCanGenerate()
{
    std::lock_guard<std::mutex> lock(hybrid_mutex_);
    if (control_mode_.phase == Mode::FAULT)
        return false;
    if (control_mode_.active_mode != Mode::VISUAL_SERVO)
        return true;
    if (!servo_stream_started_ && servo_points_.size() >= SERVO_INPUT_PRELOAD)
        servo_stream_started_ = true;
    // Preserve the 5 ms sequence. Do not invent a stationary point in the
    // middle of motion, skip a sample, or replay a burst of expired points.
    return !servo_input_fault_ && (!servo_stream_started_ || !servo_points_.empty());
}

bool AuboHardwareInterface::hybridSelectCommand(std::array<double, NUM_JOINTS>& command,
                                                std::uint64_t& fence, bool& trajectory)
{
    std::lock_guard<std::mutex> lock(hybrid_mutex_);
    if (control_mode_.phase == Mode::FAULT)
        return false;
    if (control_mode_.active_mode == Mode::ROS_CONTROL)
        return jtc_running_.load();
    trajectory = false;
    std::copy(control_mode_.hold_position.begin(), control_mode_.hold_position.end(), command.begin());
    if (control_mode_.active_mode == Mode::HOLD)
    {
        if (control_mode_.phase == Mode::DRAINING || control_mode_.phase == Mode::SWITCHING_CONTROLLER)
            fence = control_mode_.switch_id;
        return true;
    }
    if (!servo_stream_started_)
        return true;
    if (servo_input_fault_ || servo_points_.empty())
        return false;
    const auto& point = servo_points_.front();
    const double age = (ros::Time::now() - point.stamp).toSec();
    if (age < 0.0 || age > SERVO_POINT_TIMEOUT)
    {
        servo_input_fault_ = true;
        return false;
    }
    std::copy(point.position.begin(), point.position.end(), command.begin());
    servo_settled_ = point.disabled_and_settled;
    servo_points_.pop_front();
    return true;
}
} // namespace aubo_ros_control
