#!/usr/bin/python3
"""在专用固定平台 Gazebo 场景中执行完整抓放采集与验收。"""
import argparse
import json
from pathlib import Path
import sys
import threading
import time
import uuid

from .dataset import inspect_episode


def task_state(message):
    return message.split(' | ', 1)[0]


def verify_placements(initial, final, peaks, targets):
    """依据仿真世界坐标，检查每个方块的实际抬升和最终放置位置。"""
    for name, target in targets.items():
        p = final[name]
        if ((p[0]-target[0])**2+(p[1]-target[1])**2)**0.5 > 0.05 or abs(p[2]-0.12) > 0.03:
            raise RuntimeError('Block is outside its placement area: ' + name + ' ' + str(p))
        if peaks[name] < initial[name][2] + 0.05:
            raise RuntimeError('No measured lift: ' + name)


def main():
    import rospy
    from gazebo_msgs.msg import ModelStates
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        help='New verification report; existing files are never overwritten')
    parser.add_argument('--output-root', type=Path, default=Path.home()/'aubo_vla_data',
                        help='Directory for an automatically named report when --output is omitted')
    parser.add_argument('--timeout', type=float, default=300)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0 < args.timeout <= 900:
        parser.error('--timeout must be in (0, 900]')
    if args.output is None:
        root = args.output_root.expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        args.output = root/('grasp_report_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.json')
    print('验收报告：'+str(args.output), flush=True)
    # 在发出任何运动指令前创建报告，已有报告不覆盖。
    with args.output.open('x', encoding='utf-8') as output:
        output.write('{"passed": false, "stage": "starting"}\n')
    rospy.init_node('vla_grasp_recording_check', anonymous=True)
    lock = threading.Lock()
    states, poses, peaks = [], {}, {}
    models = {'red_block': [0.44, -0.25], 'green_block': [0.58, -0.25],
              'blue_block': [0.44, 0.25]}
    report = dict(passed=False, stage='preflight', episode=None)
    recording, task_started = False, False

    def state_cb(msg):
        with lock:
            states.append(msg.data)
        print('[分拣状态] ' + msg.data, flush=True)

    def model_cb(msg):
        with lock:
            for name, pose in zip(msg.name, msg.pose):
                if name in models:
                    p = pose.position
                    poses[name] = [p.x, p.y, p.z]
                    peaks[name] = max(peaks.get(name, p.z), p.z)

    subscribers = [rospy.Subscriber('/sorting/state', String, state_cb),
                   rospy.Subscriber('/gazebo/model_states', ModelStates, model_cb)]

    def wait_for(predicate, description):
        deadline = time.monotonic() + args.timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with lock:
                if predicate():
                    return
            time.sleep(0.05)
        raise RuntimeError('Timed out: ' + description)

    def call(name):
        rospy.wait_for_service(name, timeout=30)
        result = rospy.ServiceProxy(name, Trigger)()
        if not result.success:
            raise RuntimeError(name + ': ' + result.message)
        return result.message

    def operation(name):
        # 服务返回只表示命令被接受；还需等待本轮状态变为 READY 才算操作完成。
        with lock:
            begin = len(states)
        call(name)
        wait_for(lambda: any(task_state(s) in ('READY', 'ERROR', 'STOPPED')
                             for s in states[begin:]), name)
        with lock:
            final = states[-1]
        if task_state(final) != 'READY':
            raise RuntimeError('Task failed: ' + final)

    try:
        if not rospy.get_param('/use_sim_time', False):
            raise RuntimeError('This check requires a disposable Gazebo simulation')
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        wait_for(lambda: len(poses) == len(models) and bool(states), 'scene inputs')
        wait_for(lambda: task_state(states[-1]) in ('IDLE', 'READY', 'ERROR'),
                 'sorting task initialization')
        with lock:
            if task_state(states[-1]) == 'ERROR':
                raise RuntimeError('Task initialization failed: ' + states[-1])
        with lock:
            report['initial_positions'] = dict(poses)
        # 发出任务指令前拒绝已完成分拣的旧场景，避免把旧结果当作新示范。
        for name, target in models.items():
            p = report['initial_positions'][name]
            if ((p[0]-target[0])**2 + (p[1]-target[1])**2)**0.5 < 0.08:
                raise RuntimeError('Use a fresh scene: ' + name + ' is already at its destination')
        report['stage'] = 'observation'
        operation('/sorting/move_to_observation')
        # 这是写入示范数据的语言标签；实际运动仍由分拣节点规划和执行。
        instruction = 'pick up the red, green and blue blocks and place each in its matching area'
        pub = rospy.Publisher('/vla_recorder/instruction', String, queue_size=1, latch=True)
        wait_for(lambda: pub.get_num_connections() > 0, 'recorder instruction subscriber')
        pub.publish(String(data=instruction))
        time.sleep(0.5)
        report['episode'] = call('/vla_recorder/start')
        recording = True
        manifest = json.loads((Path(report['episode'])/'episode.json').read_text())
        if manifest['instruction'] != instruction:
            raise RuntimeError('Recorder did not freeze the requested instruction')
        report['stage'] = 'sorting'
        # 先开启采集，再启动分拣，确保抓取、搬运和放置过程都被记录。
        task_started = True
        operation('/sorting/start')
        task_started = False
        # 等待释放后的方块稳定，独立检查仿真世界中的抬升和放置结果，
        # 不能仅凭任务返回 READY 就给示范标记成功。
        time.sleep(1)
        with lock:
            report.update(final_positions=dict(poses), peak_heights=dict(peaks), states=list(states))
        verify_placements(report['initial_positions'], report['final_positions'],
                          report['peak_heights'], models)
        call('/vla_recorder/finish_success')
        recording = False
        report['stage'] = 'inspect'
        # 验证数据完整性，并由相邻实测 TCP 位姿生成动作差分；不回放执行动作。
        check, actions = inspect_episode(report['episode'])
        report['inspection'] = check
        if not check['passed']:
            raise RuntimeError('Episode failed inspection: ' + str(check['errors']))
        export = Path(report['episode'])/'actions.jsonl'
        with export.open('x', encoding='utf-8') as output:
            for action in actions:
                action.update(episode=str(Path(report['episode']).resolve()), outcome='success')
                output.write(json.dumps(action, allow_nan=False)+'\n')
        report.update(stage='complete', passed=True, export=str(export))
    except (Exception, KeyboardInterrupt) as exc:
        # 失败或中断时尽力停止运动并中止录制，保留证据供排查。
        report.update(failed_stage=report['stage'], stage='failed', error=str(exc))
        if task_started:
            try:
                call('/sorting/stop')
            except Exception as cleanup:
                report['stop_error'] = str(cleanup)
        if recording:
            try:
                call('/vla_recorder/abort')
            except Exception as cleanup:
                report['abort_error'] = str(cleanup)
    finally:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, ensure_ascii=False), flush=True)
        for sub in subscribers:
            sub.unregister()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
