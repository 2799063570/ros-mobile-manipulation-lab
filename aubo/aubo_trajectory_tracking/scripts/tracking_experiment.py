#!/usr/bin/env python3
"""Read-only ROS tracking capture, plus offline multi-run comparison."""
import argparse
import datetime
import hashlib
import json
import math
import queue
import sys
import threading
import time
import uuid
from pathlib import Path

# Catkin's executable relay runs source with __file__ pointing here. Import
# the sibling module itself, not an executable relay's isolated exec context.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tracking_experiment_data import CaptureSession, compare_runs, write_json


def record(args):
    # Offline analysis works without ROS installed; import ROS only for record.
    import rospy
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String, Float64MultiArray
    from rosgraph_msgs.msg import Log
    rospy.init_node('tracking_experiment_capture', anonymous=True, disable_signals=False)
    tracker = args.tracker_ns.rstrip('/')
    jog_node = args.jog_node_ns.rstrip('/')
    jog = rospy.get_param(tracker+'/jog_ns', '/cartesian_jog').rstrip('/')
    joint_topic = rospy.get_param(tracker+'/joint_states_topic', '/cartesian_jog/sdk_joint_states')
    local = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    folder = Path(args.output).expanduser().resolve()/('session_'+local.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    folder.mkdir(parents=True, exist_ok=False)
    print('采集目录：'+str(folder), flush=True)
    print('只读订阅；请在原面板手动预览/开始。Ctrl+C 仅结束采集器，不停止机械臂。', flush=True)
    events = queue.Queue(maxsize=args.queue_size)
    dropped = [0]
    drop_lock = threading.Lock()

    def enqueue(kind, data):
        event = dict(kind=kind, data=data, steady_s=time.monotonic(), ros_s=rospy.Time.now().to_sec())
        try:
            events.put_nowait(event)
        except queue.Full:
            with drop_lock:
                dropped[0] += 1

    def metadata(msg):
        try:
            enqueue('metadata', json.loads(msg.data))
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5, 'Invalid tracking run_metadata JSON')

    def joint_feedback(msg):
        enqueue('raw_joints', {'stamp_s': msg.header.stamp.to_sec(), 'seq': msg.header.seq,
                'names': list(msg.name), 'position': list(msg.position),
                'velocity': list(msg.velocity), 'effort': list(msg.effort)})

    def command(msg):
        enqueue('command', {'stamp_s': msg.header.stamp.to_sec(), 'frame': msg.header.frame_id,
                'linear': [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                'angular': [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]})

    def rosout(msg):
        if msg.name in (tracker, jog_node, '/joint_tracking_recorder'):
            enqueue('rosout', {'stamp_s': msg.header.stamp.to_sec(), 'node': msg.name,
                    'level': int(msg.level), 'text': msg.msg})

    def snapshot():
        # Whitelist motion settings; never serialize SDK credentials.
        keys = ['backend', 'control_frequency', 'joint_names', 'joint_velocity_limits',
                'joint_acceleration_limits', 'joint_jerk_limits', 'joint_lower_limits',
                'joint_upper_limits', 'joint_limit_margin', 'dls_lambda', 'dls_lambda_min',
                'adaptive_damping', 'singularity_stop', 'singularity_slow',
                'sdk_mac_buffer_target', 'sdk_mac_buffer_limit', 'sdk_diagnosis_timeout',
                'feedback_timeout', 'max_following_error', 'collision_resolution']
        jog_params = {k: rospy.get_param(jog_node+'/'+k, None) for k in keys}
        model_hash = hashlib.sha256((rospy.get_param('/robot_description', '')+'\n'+
                     rospy.get_param('/robot_description_semantic', '')).encode()).hexdigest()
        return {'parameters': rospy.get_param(tracker, {}),
                'environment': {'jog': jog_params, 'model_sha256': model_hash,
                                'joint_states_topic': joint_topic}}

    session = CaptureSession(folder, snapshot, args.tail_seconds, args.label)
    manifest = dict(schema_version=1, tracker_ns=tracker, jog_node_ns=jog_node,
                    label=args.label, queue_capacity=args.queue_size, tail_seconds=args.tail_seconds,
                    collection_only=True, created_local=local.isoformat(), queue_dropped=0)
    write_json(folder/'session.json', manifest)
    subscribers = [rospy.Subscriber(tracker+'/run_metadata', String, metadata, queue_size=20),
        rospy.Subscriber(tracker+'/run_start', Header,
                         lambda m: enqueue('start', str(m.stamp.to_nsec())), queue_size=20),
        rospy.Subscriber(tracker+'/state', String, lambda m: enqueue('state', m.data), queue_size=100),
        rospy.Subscriber(jog+'/state', String, lambda m: enqueue('backend_state', m.data), queue_size=100),
        rospy.Subscriber(tracker+'/joint_recording_state', String,
                         lambda m: enqueue('joint_recording_state', m.data), queue_size=100),
        rospy.Subscriber(joint_topic, JointState, joint_feedback, queue_size=1000,
                         tcp_nodelay=True),
        rospy.Subscriber(tracker+'/command_telemetry', TwistStamped, command, queue_size=1000),
        rospy.Subscriber('/rosout', Log, rosout, queue_size=1000)]
    for topic, kind in [('samples', 'tcp'), ('joint_samples', 'joints'), ('servo_samples', 'servo')]:
        subscribers.append(rospy.Subscriber(tracker+'/'+topic, Float64MultiArray,
            lambda msg, k=kind: enqueue(k, list(msg.data)), queue_size=1000, tcp_nodelay=True))
    started = time.monotonic()
    collector_started_ros_ns = rospy.Time.now().to_nsec()
    last_flush = started
    announced = 0
    try:
        while not rospy.is_shutdown():
            try:
                event = events.get(timeout=.1)
                if not (args.only_new_runs and event['kind']=='start' and
                        int(event['data'])!=0 and int(event['data'])<collector_started_ros_ns):
                    session.consume(event, dropped[0])
            except queue.Empty:
                pass
            session.tick(time.monotonic(), dropped[0])
            if len(session.closed) > announced:
                for result in session.closed[announced:]:
                    metric = result['tcp']['running']['total_mm']
                    print('已保存 %s：%s；运行段峰值=%s mm，RMS=%s mm；质量标记=%s'%(
                        result['run_id'], result['state'], metric['peak_abs'], metric['rms'],
                        ','.join(result['quality']) or 'complete'), flush=True)
                announced = len(session.closed)
            if time.monotonic()-last_flush >= 1:
                if session.active:
                    session.active.flush()
                manifest['queue_dropped'] = dropped[0]
                write_json(folder/'session.json', manifest)
                last_flush = time.monotonic()
            if args.duration and time.monotonic()-started >= args.duration:
                break
            # Ignore an old completed latch with zero samples when joining late.
            observed = sum(r['tcp']['all']['total_mm']['n'] > 0 for r in session.closed)
            if args.runs and observed >= args.runs:
                break
    finally:
        for sub in subscribers:
            sub.unregister()
        # Drain callbacks already received before closing files.
        while True:
            try:
                session.consume(events.get_nowait(), dropped[0])
            except queue.Empty:
                break
        session.finish('INTERRUPTED: collector stopped', dropped[0])
        manifest.update(queue_dropped=dropped[0], closed_runs=len(session.closed))
        write_json(folder/'session.json', manifest)
        result = compare_runs([folder], folder/'comparison', plots=not args.no_plots)
        print('采集结束；对比报告：'+str(folder/'comparison/report.md'), flush=True)
        if result['issues']:
            print('\n'.join(result['issues']), file=sys.stderr)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='只读采集轨迹实验并离线比较；不会调用运动服务或设置参数。')
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('record', help='持续订阅并按 run_start 自动分轮；Ctrl+C 结束采集')
    capture.add_argument('--output', required=True, help='采集根目录；自动创建独立 session 子目录')
    capture.add_argument('--label', default='', help='本次会话说明，例如 P2_FF010')
    capture.add_argument('--tracker-ns', default='/trajectory_tracking')
    capture.add_argument('--jog-node-ns', default='/aubo_cartesian_jog')
    capture.add_argument('--tail-seconds', type=float, default=2.5)
    capture.add_argument('--queue-size', type=int, default=10000)
    capture.add_argument('--runs', type=int, default=0, help='采到 N 轮有 TCP 的记录后退出，0 为持续采集')
    capture.add_argument('--duration', type=float, default=0, help='最多采集的真实秒数，0 为不限')
    capture.add_argument('--no-plots', action='store_true')
    capture.add_argument('--only-new-runs', action='store_true', help='忽略启动前已有的锁存 run_start，仅采集新的运行')
    compare = commands.add_parser('compare', help='离线比较 session 或 run 目录，可提供多个目录')
    compare.add_argument('inputs', nargs='+')
    compare.add_argument('--output', required=True)
    compare.add_argument('--no-plots', action='store_true')
    study = commands.add_parser('pid-study', help='离线统计偏差/速度误差，回放 I/D 候选修正量（不预测闭环误差）')
    study.add_argument('inputs', nargs='+')
    study.add_argument('--output', required=True)
    study.add_argument('--ki-grid', nargs='+', type=float, default=[0., .005, .01, .02, .05, .1])
    study.add_argument('--kd-grid', nargs='+', type=float, default=[0., .01, .02, .05, .1])
    # rosrun/roslaunch may append remapping arguments; use rospy only in record.
    if argv is None:
        argv = [v for v in sys.argv[1:] if ':=' not in v]
    args = parser.parse_args(argv)
    if args.command == 'pid-study':
        if any(not math.isfinite(v) or v<0 or v>2 for v in args.ki_grid) or any(
                not math.isfinite(v) or v<0 or v>1 for v in args.kd_grid):
            parser.error('Ki 必须在 0..2，Kd 必须在 0..1，且为有限数')
        from tracking_pid_study import study_runs
        result = study_runs(args.inputs, args.output, sorted(set(args.ki_grid)), sorted(set(args.kd_grid)))
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result['runs'] else 1
    if args.command == 'compare':
        result = compare_runs(args.inputs, args.output, plots=not args.no_plots)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result['runs'] else 1
    if (not math.isfinite(args.tail_seconds) or args.tail_seconds < 0 or args.queue_size < 1 or
            not math.isfinite(args.duration) or args.duration < 0 or args.runs < 0):
        parser.error('tail/duration/runs 必须非负且有限，queue-size 必须为正')
    return record(args)


if __name__ == '__main__':
    sys.exit(main())
