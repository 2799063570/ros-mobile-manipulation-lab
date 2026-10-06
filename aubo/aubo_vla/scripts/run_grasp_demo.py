#!/usr/bin/python3
"""自动启动场景、录制抓放示范、验收结果并导出动作数据。"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid


def free_ports():
    """获取两个当前可用的本机端口，分别供 ROS 和 Gazebo 主服务使用。"""
    with socket.socket() as ros, socket.socket() as gazebo:
        ros.bind(('127.0.0.1', 0))
        gazebo.bind(('127.0.0.1', 0))
        return ros.getsockname()[1], gazebo.getsockname()[1]


def stop_process(process, interrupt_timeout=15, terminate_timeout=5, kill_timeout=3):
    if process is None:
        return
    # 每个子进程都有独立进程组；只清理本次创建的进程，不停止其他场景。
    for sig, timeout in [(signal.SIGINT, interrupt_timeout),
                         (signal.SIGTERM, terminate_timeout), (signal.SIGKILL, kill_timeout)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    import rospy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=Path.home()/'aubo_vla_data')
    parser.add_argument('--headless', action='store_true', help='Disable Gazebo and RViz windows')
    parser.add_argument('--gui', choices=['true', 'false'], default='true')
    parser.add_argument('--rviz', choices=['true', 'false'], default='true')
    parser.add_argument('--keep-open', action='store_true', help='Keep scene visible until Ctrl+C after success')
    parser.add_argument('--timeout', type=float, default=300, help='Workflow stage timeout in seconds')
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0 < args.timeout <= 900:
        parser.error('--timeout must be in (0, 900]')
    # 使用系统 Python 加载已安装模块；源码运行时补充本包的模块路径。
    source = Path(__file__).resolve().parents[1]/'src'
    if source.is_dir():
        sys.path.insert(0, str(source))
    from aubo_vla import grasp_workflow

    root = args.output_root.expanduser().resolve()
    # 每次运行建立独立日志目录；示范 episode 另存于同一个输出根目录。
    run = root/('run_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    run.mkdir(parents=True, exist_ok=False)
    ros_port, gazebo_port = free_ports()
    env = dict(os.environ)
    # 部分 ROS 工具通过 env 查找 python3，优先系统路径，
    # 避免终端已激活的模型或 Conda 环境影响控制器及 ROS 依赖。
    env['PATH'] = '/usr/bin:/bin:'+env.get('PATH', '')
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % ros_port,
               GAZEBO_MASTER_URI='http://127.0.0.1:%d' % gazebo_port,
               ROS_HOSTNAME='localhost')
    env.pop('ROS_IP', None)
    module_root = str(Path(grasp_workflow.__file__).resolve().parents[1])
    env['PYTHONPATH'] = module_root+os.pathsep+env.get('PYTHONPATH', '')
    metadata = dict(stage='starting', passed=False, run=str(run),
                    ros_master=env['ROS_MASTER_URI'], gazebo_master=env['GAZEBO_MASTER_URI'])
    (run/'run.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    launch, worker = None, None
    result = 1
    print('实例目录：'+str(run), flush=True)
    try:
        with (run/'launch.log').open('w') as log:
            # 启动场景及业务节点；它们继承上面设置的独立主服务地址。
            launch = subprocess.Popen([
                'roslaunch', '--sigint-timeout=5', '--sigterm-timeout=2',
                'aubo_vla', 'grasp_recording_gazebo.launch',
                'gui:='+('false' if args.headless else args.gui),
                'rviz:='+('false' if args.headless else args.rviz), 'output_root:='+str(root)],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+60
            # 主服务端口就绪后启动工作流；控制器及场景的就绪由工作流继续检查。
            while True:
                if launch.poll() is not None:
                    raise RuntimeError('Scene launch exited; see '+str(run/'launch.log'))
                try:
                    with socket.create_connection(('127.0.0.1', ros_port), timeout=0.3):
                        break
                except OSError:
                    if time.monotonic() > deadline:
                        raise RuntimeError('ROS master startup timed out')
                    time.sleep(0.2)
            print('场景已启动，等待控制器就绪后自动进入观察位并开始抓放。', flush=True)
            metadata['stage'] = 'workflow'
            (run/'run.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
            worker = subprocess.Popen([
                sys.executable, '-m', 'aubo_vla.grasp_workflow',
                '--output', str(run/'report.json'), '--timeout', str(args.timeout)],
                env=env, start_new_session=True)
            # 同时监视工作流与场景，场景提前退出时停止采集流程。
            while worker.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError('Scene exited during the workflow')
                time.sleep(0.2)
            result = worker.returncode
            # 报告记录真实任务结果及数据完整性，成功后按需保留可视化窗口。
            report = json.loads((run/'report.json').read_text())
            metadata.update(stage='complete' if result == 0 else 'failed', passed=report['passed'],
                            episode=report['episode'], report=str(run/'report.json'))
            (run/'run.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
            if result == 0:
                print('抓放与数据检查通过。示范：'+report['episode'], flush=True)
                if args.keep_open:
                    print('场景保留；按 Ctrl+C 关闭本次实例。', flush=True)
                    while launch.poll() is None:
                        time.sleep(0.5)
            else:
                print('实例未通过，查看报告：'+str(run/'report.json'), flush=True)
    except KeyboardInterrupt:
        if metadata['stage'] != 'complete':
            metadata.update(stage='interrupted', error='operator interrupted')
            result = 130
    except Exception as exc:
        metadata.update(stage='failed', passed=False, error=str(exc))
        result = 1
        print(str(exc), file=sys.stderr, flush=True)
    finally:
        # 在外层 roslaunch 默认的 15 秒退出宽限期内完成子进程清理。
        stop_process(worker, interrupt_timeout=2, terminate_timeout=1, kill_timeout=1)
        stop_process(launch, interrupt_timeout=8, terminate_timeout=1, kill_timeout=1)
        (run/'run.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        print('报告与启动日志：'+str(run), flush=True)
    return result


if __name__ == '__main__':
    sys.exit(main())
