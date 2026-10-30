#!/usr/bin/env python3
"""Summarize MoveIt execution, tracking error, and AUBO MAC flow from a rosbag."""

import argparse
import json
import math
from pathlib import Path

import rosbag


FLOW_TOPIC = "/aubo_i5/aubo_hw_node/flow_diagnostics"
STATE_TOPIC = "/aubo_i5/aubo_i5_controller/state"
GOAL_TOPIC = "/aubo_i5/aubo_i5_controller/follow_joint_trajectory/goal"
RESULT_TOPIC = "/aubo_i5/aubo_i5_controller/follow_joint_trajectory/result"
MOVE_GROUP_GOAL_TOPIC = "/move_group/goal"
EXECUTE_GOAL_TOPIC = "/execute_trajectory/goal"
ROSOUT_TOPIC = "/rosout"


def numeric(values, key, default=0.0):
    try:
        value = float(values[key])
        return value if math.isfinite(value) else default
    except (KeyError, TypeError, ValueError):
        return default


def trajectory_peaks(points):
    peaks = {"velocity_rad_s": 0.0, "acceleration_rad_s2": 0.0,
             "jerk_rad_s3_sampled": 0.0}
    previous = None
    for point in points:
        if point.velocities:
            peaks["velocity_rad_s"] = max(
                [peaks["velocity_rad_s"]] + [abs(v) for v in point.velocities])
        if point.accelerations:
            peaks["acceleration_rad_s2"] = max(
                [peaks["acceleration_rad_s2"]] + [abs(a) for a in point.accelerations])
        timestamp = point.time_from_start.to_sec()
        if previous and point.accelerations and previous[1] and timestamp > previous[0]:
            dt = timestamp - previous[0]
            jerk = [abs(a - old) / dt
                    for a, old in zip(point.accelerations, previous[1])]
            peaks["jerk_rad_s3_sampled"] = max(
                [peaks["jerk_rad_s3_sampled"]] + jerk)
        previous = (timestamp, list(point.accelerations))
    return peaks


def execution_segments(states, flow, goals, results, path_tolerance, bag_start):
    moving = []
    for stamp, msg in states:
        if any(abs(value) > 0.005 for value in msg.desired.velocities):
            moving.append((stamp, msg))
    segments = []
    for row in moving:
        if not segments or row[0] - segments[-1][-1][0] > 0.25:
            segments.append([row])
        else:
            segments[-1].append(row)

    reports = []
    for index, segment in enumerate(segments):
        start, end = segment[0][0], segment[-1][0]
        names = segment[0][1].joint_names
        worst = {name: (0.0, None) for name in names}
        desired_velocity = {name: 0.0 for name in names}
        desired_acceleration = {name: 0.0 for name in names}
        for stamp, msg in segment:
            for joint_index, name in enumerate(msg.joint_names):
                error = abs(msg.error.positions[joint_index]) if joint_index < len(msg.error.positions) else 0.0
                if error > worst.get(name, (0.0, None))[0]:
                    worst[name] = (error, stamp)
                if joint_index < len(msg.desired.velocities):
                    desired_velocity[name] = max(
                        desired_velocity.get(name, 0.0),
                        abs(msg.desired.velocities[joint_index]))
                if joint_index < len(msg.desired.accelerations):
                    desired_acceleration[name] = max(
                        desired_acceleration.get(name, 0.0),
                        abs(msg.desired.accelerations[joint_index]))
        worst_joint = max(worst, key=lambda name: worst[name][0])
        segment_flow = [row for row in flow
                        if start - 0.15 <= row["time"] <= end + 0.15 and
                        row["values"].get("stream_active") == "true"]
        flow_values = [row["values"] for row in segment_flow]
        min_mac = [numeric(v, "mac_active_window_min_scalars", -1.0) / 6.0
                   for v in flow_values
                   if numeric(v, "mac_active_window_min_scalars", -1.0) >= 0]
        goal = goals[index] if index < len(goals) else None
        result = results[index] if index < len(results) else None
        reports.append({
            "start_s": round(start - bag_start, 3),
            "end_s": round(end - bag_start, 3),
            "duration_s": round(end - start, 3),
            "result_status": result.status.status if result else None,
            "result_error_code": result.result.error_code if result else None,
            "trajectory_knot_peaks": trajectory_peaks(goal.points) if goal else None,
            "controller_desired_max_velocity_rad_s": max(desired_velocity.values(), default=0.0),
            "controller_desired_max_acceleration_rad_s2": max(desired_acceleration.values(), default=0.0),
            "max_tracking_error_joint": worst_joint,
            "max_tracking_error_rad": worst[worst_joint][0],
            "max_tracking_error_deg": round(worst[worst_joint][0] * 180.0 / math.pi, 3),
            "max_tracking_error_time_s": round(worst[worst_joint][1] - bag_start, 3)
                if worst[worst_joint][1] is not None else None,
            "exceeds_path_tolerance": worst[worst_joint][0] > path_tolerance,
            "flow_samples": len(segment_flow),
            "max_mac_query_ms": round(max(
                [numeric(v, "diagnosis_max_ms") for v in flow_values] or [0.0]), 3),
            "max_sdk_send_ms": round(max(
                [numeric(v, "send_max_ms") for v in flow_values] or [0.0]), 3),
            "max_feed_loop_ms": round(max(
                [numeric(v, "feed_loop_max_ms") for v in flow_values] or [0.0]), 3),
            "min_active_mac_points": min(min_mac) if min_mac else None,
            "max_ros_queue_points": max(
                [numeric(v, "queue_window_max_points") for v in flow_values] or [0.0]),
            "software_queue_warning_points": numeric(
                flow_values[-1] if flow_values else {}, "queue_backlog_warning_points",
                numeric(flow_values[-1] if flow_values else {}, "mac_target_points", 0.0)),
            "max_ros_queue_nominal_horizon_ms": round(max(
                [numeric(v, "queue_window_max_nominal_horizon_ms",
                         numeric(v, "queue_window_max_points") * 5.0)
                 for v in flow_values] or [0.0]), 3),
            "low_water_samples_below_5_points": int(sum(
                numeric(v, "mac_low_water_samples_below_5_points") for v in flow_values)),
            "empty_with_pending_queue_samples": int(sum(
                numeric(v, "mac_empty_with_pending_queue_samples") for v in flow_values)),
        })
    return reports


def analyze(path, path_tolerance):
    flow = []
    states = []
    goals = []
    results = []
    move_group_goals = []
    planning_requests = []
    execute_goals = []
    handeye_status = []
    with rosbag.Bag(str(path)) as bag:
        bag_start = bag.get_start_time()
        duration = bag.get_end_time() - bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
                topics=[FLOW_TOPIC, STATE_TOPIC, GOAL_TOPIC, RESULT_TOPIC,
                        MOVE_GROUP_GOAL_TOPIC, EXECUTE_GOAL_TOPIC, ROSOUT_TOPIC]):
            if topic == FLOW_TOPIC:
                for status in msg.status:
                    if status.name != "Aubo hardware motion flow":
                        continue
                    values = {item.key: item.value for item in status.values}
                    flow.append({"time": stamp.to_sec(), "level": status.level,
                                 "message": status.message, "values": values})
            elif topic == STATE_TOPIC:
                states.append((stamp.to_sec(), msg))
            elif topic == GOAL_TOPIC:
                goals.append(msg.goal.trajectory)
            elif topic == RESULT_TOPIC:
                results.append(msg)
            elif topic == MOVE_GROUP_GOAL_TOPIC:
                move_group_goals.append(msg)
                request = msg.goal.request
                planning_requests.append({
                    "goal_id": msg.goal_id.id,
                    "time_ros_s": stamp.to_sec(),
                    "group_name": request.group_name,
                    "requested_velocity_scaling": request.max_velocity_scaling_factor,
                    "requested_acceleration_scaling": request.max_acceleration_scaling_factor,
                })
            elif topic == EXECUTE_GOAL_TOPIC:
                execute_goals.append(msg)
            elif topic == ROSOUT_TOPIC and msg.name == "/handeye_yolo_verify":
                handeye_status.append(msg.msg)

    report = {
        "bag": str(path),
        "duration_s": round(duration, 3),
        "hardware_flow_samples": len(flow),
        "hardware_trace_prefixes": sorted({row['values'].get('motion_trace_prefix', '') for row in flow} - {''}),
        "controller_state_samples": len(states),
        "trajectory_goals": len(goals),
        "trajectory_results": len(results),
        "motion_entrypoints": {
            "move_group_goal_count": len(move_group_goals),
            "move_group_goal_ids": [goal.goal_id.id for goal in move_group_goals],
            # Use recorded requests, never infer runtime values from RViz defaults.
            "move_group_requests": planning_requests,
            "execute_trajectory_goal_count": len(execute_goals),
            "handeye_yolo_verify_status": handeye_status,
        },
    }

    if flow:
        all_values = [row["values"] for row in flow]
        get_max = lambda key: max(numeric(v, key) for v in all_values)
        get_sum = lambda key: sum(numeric(v, key) for v in all_values)
        mac_min = [numeric(v, "mac_active_window_min_scalars", -1.0)
                   for v in all_values]
        mac_min = [value for value in mac_min if value >= 0]
        report["hardware_flow"] = {
            "trajectory_execution_modes": sorted({v.get("trajectory_execution_mode", "jtc") for v in all_values}),
            "controller_state_time_bases": sorted({v["controller_state_time_basis"] for v in all_values if v.get("controller_state_time_basis")}),
            "jtc_queue_target_points": numeric(all_values[-1], "jtc_queue_target_points", -1),
            "jtc_backpressure_waits_total": get_max("jtc_backpressure_waits_total"),
            "mac_target_points": numeric(all_values[-1], "mac_target_points"),
            "mac_target_coverage_ms": numeric(all_values[-1], "mac_target_coverage_ms", -1.0),
            "observed_min_active_mac_points": min(mac_min) / 6.0 if mac_min else None,
            "observed_max_mac_points": get_max("mac_window_max_scalars") / 6.0,
            "max_ros_queue_points": get_max("queue_window_max_points"),
            "software_queue_warning_points": numeric(
                all_values[-1], "queue_backlog_warning_points",
                numeric(all_values[-1], "mac_target_points", 0.0)),
            "max_ros_queue_nominal_horizon_ms": round(max(
                [numeric(v, "queue_window_max_nominal_horizon_ms",
                         numeric(v, "queue_window_max_points") * 5.0)
                 for v in all_values] or [0.0]), 3),
            "max_oldest_queued_command_age_ms": get_max("queue_oldest_command_age_ms"),
            "command_queue_duration_limit_ms": numeric(all_values[-1], "max_command_queue_duration_ms", -1),
            "command_queue_limit_exceeded": any(v.get("command_queue_limit_exceeded") == "true" for v in all_values),
            "feedback_sources": sorted({v["feedback_source"] for v in all_values if v.get("feedback_source")}),
            "feedback_max_sample_age_ms": get_max("feedback_sample_age_ms"),
            "feedback_ignored_push_samples_total": get_max("feedback_ignored_push_samples_total"),
            "feedback_query_failures_total": get_max("feedback_query_failures_total"),
            "feedback_query_max_ms": get_max("feedback_query_max_ms"),
            "feedback_timeout_latched": any(v.get("feedback_timeout_latched") == "true" for v in all_values),
            "slow_mac_query_samples_over_50ms": int(get_sum("mac_slow_query_samples_over_50ms")),
            "mac_query_exceeded_reported_coverage_samples": int(get_sum("mac_query_exceeded_reported_coverage_samples")),
            "max_mac_query_ms": round(get_max("diagnosis_max_ms"), 3),
            "max_sdk_send_ms": round(get_max("send_max_ms"), 3),
            "max_feed_loop_ms": round(get_max("feed_loop_max_ms"), 3),
            "max_controller_loop_period_ms": round(
                get_max("controller_loop_period_max_ms"), 3),
            "mac_low_water_samples_below_5_points": int(
                get_sum("mac_low_water_samples_below_5_points")),
            "mac_empty_with_pending_queue_samples": int(
                get_sum("mac_empty_with_pending_queue_samples")),
            "mac_active_empty_samples": int(get_sum("mac_active_empty_samples")),
            "mac_data_interrupt_warning_samples": int(get_sum("mac_data_interrupt_warning_samples")),
            "stream_fault_latched": any(v.get("stream_fault_latched") == "true" for v in all_values),
            "output_sample_peaks": {
                "velocity_rad_s": get_max("command_velocity_max_rad_s"),
                "acceleration_rad_s2": get_max("command_acceleration_max_rad_s2"),
                "jerk_rad_s3": get_max("command_jerk_max_rad_s3"),
            },
            "mac_above_target_samples": int(get_sum("mac_above_target_samples")),
            "diagnosis_failures": int(get_sum("diagnosis_failures")),
            "send_failures": int(get_sum("send_failures")),
            "queue_full_total": int(get_max("queue_full_total")),
            "queue_full_events_in_capture": int(get_sum("queue_full_window")),
            "tcp_info_available": any(v.get("tcp_info_valid") == "true" for v in all_values),
            "tcp_retransmissions_in_capture": int(get_sum("tcp_retransmissions_window")),
            "tcp_max_rtt_ms": get_max("tcp_max_rtt_ms"),
            "tcp_max_rto_ms": get_max("tcp_max_rto_ms"),
        }
    else:
        report["hardware_flow"] = None

    if goals:
        report["planned_trajectory_peaks"] = [
            {"duration_s": round(t.points[-1].time_from_start.to_sec(), 4)
             if t.points else 0.0,
             **trajectory_peaks(t.points)} for t in goals]
    else:
        report["planned_trajectory_peaks"] = None

    if states:
        joint_names = states[0][1].joint_names
        max_error = {name: 0.0 for name in joint_names}
        max_error_time = {name: None for name in joint_names}
        max_desired_v = {name: 0.0 for name in joint_names}
        max_desired_a = {name: 0.0 for name in joint_names}
        active_samples = 0
        for stamp, msg in states:
            sample_active = False
            for i, name in enumerate(msg.joint_names):
                error = abs(msg.error.positions[i]) if i < len(msg.error.positions) else 0.0
                desired_v = abs(msg.desired.velocities[i]) if i < len(msg.desired.velocities) else 0.0
                desired_a = abs(msg.desired.accelerations[i]) if i < len(msg.desired.accelerations) else 0.0
                max_error[name] = max(max_error.get(name, 0.0), error)
                if error >= max_error[name]:
                    max_error_time[name] = stamp
                max_desired_v[name] = max(max_desired_v.get(name, 0.0), desired_v)
                max_desired_a[name] = max(max_desired_a.get(name, 0.0), desired_a)
                if desired_v > 1e-4 or desired_a > 1e-4:
                    sample_active = True
            active_samples += int(sample_active)
        worst_joint = max(max_error, key=max_error.get)
        report["controller_tracking"] = {
            "path_tolerance_rad": path_tolerance,
            "max_error_rad_by_joint": max_error,
            "max_error_deg_by_joint": {
                name: round(value * 180.0 / math.pi, 3)
                for name, value in max_error.items()},
            "worst_joint": worst_joint,
            "worst_error_rad": max_error[worst_joint],
            "worst_error_time": max_error_time[worst_joint],
            "joints_exceeding_path_tolerance": [
                name for name, value in max_error.items() if value > path_tolerance],
            "max_desired_velocity_rad_s_by_joint": max_desired_v,
            "max_desired_acceleration_rad_s2_by_joint": max_desired_a,
            "moving_state_samples": active_samples,
        }
        if flow:
            nearest = min(flow, key=lambda row: abs(
                row["time"] - (max_error_time[worst_joint] or row["time"])) )
            report["controller_tracking"]["flow_sample_near_worst_error"] = {
                "delta_ms": round(abs(nearest["time"] -
                    (max_error_time[worst_joint] or nearest["time"])) * 1000.0, 1),
                "message": nearest["message"],
                "mac_current_scalars": numeric(nearest["values"], "mac_current_scalars", -1),
                "queue_current_points": numeric(nearest["values"], "queue_current_points", -1),
                "diagnosis_max_ms": numeric(nearest["values"], "diagnosis_max_ms", -1),
                "send_max_ms": numeric(nearest["values"], "send_max_ms", -1),
            }
    else:
        report["controller_tracking"] = None

    report["controller_results"] = [
        {"status": result.status.status,
         "status_text": result.status.text,
         "error_code": result.result.error_code,
         "error_string": result.result.error_string}
        for result in results]
    report["execution_segments"] = execution_segments(
        states, flow, goals, results, path_tolerance, bag_start)

    findings = []
    hw = report["hardware_flow"]
    if move_group_goals and not execute_goals:
        findings.append("轨迹由 /move_group action 发起，没有 /execute_trajectory goal；这次未走 handeye_yolo_verify/move 验证入口。")
    if hw is None:
        findings.append("缺少硬件流量诊断话题；该 bag 不是由带新诊断的硬件节点录制。")
    else:
        if "precomputed" in hw["trajectory_execution_modes"]:
            # Historical bags only: the executable now supports JTC exclusively.
            findings.append("本次为预生成轨迹模式；controller/state 的目标相位由 MAC 水位估计，不能把其误差数值直接与 JTC 墙钟目标误差比较，也不能当作振动幅度。")
        if hw["feedback_timeout_latched"]:
            findings.append("有效关节反馈超时，已锁定停流；旧推送和迟到查询回复不刷新采样年龄。")
        if hw["command_queue_limit_exceeded"]:
            findings.append("待发指令时长或实际等待年龄超过上限，已锁定停流，禁止丢点追赶。")
        if "query_latched" in hw["feedback_sources"]:
            findings.append("独立路点查询已接管反馈，随后推送不再覆盖查询状态；检查查询耗时和采样年龄。")
        if hw["tcp_retransmissions_in_capture"]:
            findings.append("机械臂 SDK 连接出现 TCP 重传；对照慢 RPC 时刻检查链路，柜端发送的重传仍需抓包确认。")
        if hw["mac_data_interrupt_warning_samples"]:
            findings.append("控制柜报告 MAC 数据中断标志；驱动已锁定停流，需检查柜内日志后重启。")
        if hw["mac_active_empty_samples"]:
            findings.append("已建立的连续流观测到 MAC 空水位；驱动禁止继续发送积压运动点。")
        if hw["stream_fault_latched"]:
            findings.append("驱动记录到连续流/轨迹保护故障，结合 /rosout 的首条故障定位原因。")
        if hw["diagnosis_failures"] or hw["send_failures"] or hw["queue_full_events_in_capture"]:
            findings.append("检测到 MAC 查询/发送失败或 ROS 运动队列满，优先排查通信供点链路。")
        if hw["mac_empty_with_pending_queue_samples"]:
            findings.append("出现 MAC 为空但 ROS 队列仍有待发点的采样，疑似缓冲欠载；需结合动作时刻复核。")
        if hw["mac_low_water_samples_below_5_points"]:
            findings.append("运动中多次观测到 MAC 低于 5 个路点（当前队列同时有待发点）；存在缓冲余量不足风险。")
        if hw["max_feed_loop_ms"] > hw["mac_target_coverage_ms"] > 0:
            findings.append("最长供点循环间隔超过 MAC 目标覆盖时长，存在供点断档风险；核对同一时刻空水位及柜内中断标志。")
        if hw["max_ros_queue_points"] > hw["software_queue_warning_points"] > 0:
            findings.append("ROS 软件运动队列峰值超过 MAC 目标点数；这些待发点会增加命令滞后，应结合跟随误差出现时刻检查。")
        if hw["mac_above_target_samples"]:
            findings.append("观测到 MAC 高于补点目标，检查 SDK 水位单位与控制柜容量；这不是单独的溢出证明。")
        if max(hw["max_mac_query_ms"], hw["max_sdk_send_ms"], hw["max_feed_loop_ms"]) > 20.0:
            findings.append("存在超过 20 ms 的查询/发送/喂点周期样本；请与本次 MAC 覆盖时长和跟随误差的时间相关性一起判断。")
    tracking = report["controller_tracking"]
    if tracking and tracking["joints_exceeding_path_tolerance"]:
        findings.append("控制器反馈误差超过路径容差；比较目标加速度与同一时刻的 MAC/RPC 指标。")
    if tracking and not tracking["joints_exceeding_path_tolerance"]:
        findings.append("采样到的关节误差未超过给定容差；再结合现场抖动和高频柜内日志判断。")
        if tracking["worst_error_rad"] >= 0.8 * path_tolerance:
            findings.append("最大关节跟随误差已达到路径容差的 80% 以上，虽然动作成功，仍有明显滞后。")
    report["findings"] = findings
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--path-tolerance", type=float, default=0.15,
                        help="JointTrajectoryController trajectory tolerance in radians")
    parser.add_argument("--json", type=Path, help="also save the report as JSON")
    args = parser.parse_args()
    report = analyze(args.bag, args.path_tolerance)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.json:
        args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
