#!/usr/bin/env python3
"""Explicitly actuated, bounded tracking experiment matrix through existing services.

Unlike tracking_experiment.py (read-only), --execute calls preview/start/stop and
changes only the four outer-loop tuning parameters. Never calls reset or HOME.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import math
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tracking_experiment_data import compare_runs, write_json

CASES = [('baseline', 0., 0.), ('I002', .02, 0.), ('I005', .05, 0.),
         ('D001', 0., .01), ('D002', 0., .02), ('I002_D002', .02, .02)]
TUNING = ('kp', 'ki', 'kd', 'acceleration_ff_time')


def build_plan(repeats):
    result = []
    for repeat in range(repeats):
        order = CASES if repeat % 2 == 0 else list(reversed(CASES))
        for label, ki, kd in order:
            result.append(dict(case=label, repeat=repeat+1, ki=ki, kd=kd))
    return result


def remaining_plan(previous):
    allowed={(label,ki,kd) for label,ki,kd in CASES}
    completed={(row['case'],row['repeat']) for row in previous['outcomes'] if not row['quality']
               and row['state'].startswith('COMPLETED:')}
    result=[]
    for row in previous['plan']:
        if (row['case'],row['ki'],row['kd']) not in allowed or row['repeat'] not in (1,2,3):
            raise RuntimeError('Resume plan contains an unsupported case')
        if (row['case'],row['repeat']) not in completed:
            result.append(row)
    return result


def load_resume_manifest(path):
    path=Path(path)
    completion=path.with_name('completion.json')
    if completion.exists() and json.loads(completion.read_text()).get('state')=='COMPLETE':
        raise RuntimeError('This matrix already completed after recovery; use its completion manifest')
    return json.loads(path.read_text())


def validate_profile(parameters, backend, expected_backend, use_sim_time, require_real):
    if backend != expected_backend:
        raise RuntimeError('Execution backend differs from the explicitly requested backend')
    if not parameters.get('check_feasibility', True):
        raise RuntimeError('Full-curve feasibility checks must remain enabled')
    if parameters.get('shape') != 'circle' or int(parameters.get('cycles', 0)) != 1:
        raise RuntimeError('Batch runner requires a one-cycle closed circle')
    if expected_backend == 'sdk':
        if use_sim_time or not require_real:
            raise RuntimeError('Real SDK experiments require wall ROS time and require_real_robot=true')
        expected = {'radius': .12, 'duration': 40., 'max_speed': .07, 'kp': 2.,
                    'acceleration_ff_time': .1, 'correction_fraction': .2}
        for key, value in expected.items():
            actual=float(parameters.get(key,float('nan')))
            if not math.isfinite(actual) or abs(actual-value)>1e-8:
                raise RuntimeError('Real task profile differs at '+key)
        if parameters.get('plane') != 'xy' or float(parameters.get('max_extent', 1))>.25:
            raise RuntimeError('Real task geometry differs from the tested profile')
    for key, limit in [('kp', 3.), ('ki', 2.), ('kd', 1.), ('acceleration_ff_time', .25)]:
        value = float(parameters.get(key, float('nan')))
        if not math.isfinite(value) or value < 0 or value > limit:
            raise RuntimeError('Invalid initial tuning value '+key)


def execute(args):
    import rospy
    import rosgraph
    from std_msgs.msg import String, Header, Float64MultiArray
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Path as RosPath
    from std_srvs.srv import Trigger, SetBool

    rospy.init_node('tracking_experiment_runner', anonymous=True, disable_signals=False)
    lock_file = open('/tmp/aubo_tracking_experiment_runner.lock', 'a')
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    tracker, jog_node = args.tracker_ns.rstrip('/'), args.jog_node_ns.rstrip('/')
    initial = rospy.get_param(tracker)
    jog = initial.get('jog_ns', '/cartesian_jog').rstrip('/')
    backend = rospy.get_param(jog_node+'/backend')
    validate_profile(initial, backend, args.backend, rospy.get_param('/use_sim_time', False),
                     rospy.get_param(jog_node+'/require_real_robot', False))
    previous=load_resume_manifest(args.resume) if args.resume else None
    if previous is not None:
        for key,value in previous['initial_parameters'].items():
            if key not in TUNING and initial.get(key)!=value:
                raise RuntimeError('Resume trajectory differs at '+key)
    plan = remaining_plan(previous) if previous is not None else build_plan(args.repeats)
    if not plan:raise RuntimeError('No remaining trials')
    local = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    output = Path(args.output).expanduser().resolve()/('real_pid_'+local.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    output.mkdir(parents=True)
    manifest = dict(schema_version=1, state='PREPARING', backend=backend, initial_parameters=initial,
                    plan=plan, outcomes=[], created_local=local.isoformat(), user_authorized_motion=True)
    if args.resume:manifest['resume_from']=str(Path(args.resume).resolve())
    write_json(output/'experiment.json', manifest)
    journal = (output/'runner_events.jsonl').open('w', buffering=1)
    def log(kind, **data):
        journal.write(json.dumps(dict(kind=kind, local_time=datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=8))).isoformat(), **data), ensure_ascii=False)+'\n')
        print(json.dumps(dict(kind=kind, **data), ensure_ascii=False), flush=True)
    log('plan', output=str(output), trials=len(plan), cases=CASES)
    shared, shared_lock = {}, threading.RLock()
    def store(name, data):
        with shared_lock:
            shared[name] = (time.monotonic(), data)
    def feedback(msg):
        if len(msg.name)!=len(msg.position) or not all(math.isfinite(v) for v in msg.position):
            return
        store('feedback', dict(stamp=msg.header.stamp.to_sec(), names=list(msg.name), q=list(msg.position)))
    def reference(msg):
        if msg.poses:
            p=msg.poses[0].pose.position;q=msg.poses[0].pose.orientation
            store('reference', dict(stamp=msg.header.stamp.to_nsec(), origin=[p.x,p.y,p.z],
                                   orientation=[q.x,q.y,q.z,q.w]))
    def read(name, max_age=None):
        with shared_lock:
            entry=shared.get(name)
        if entry is None or max_age is not None and time.monotonic()-entry[0]>max_age:
            raise RuntimeError('Missing/stale '+name)
        return entry[1]
    subscribers=[rospy.Subscriber(tracker+'/state',String,lambda m:store('state',m.data),queue_size=50),
        rospy.Subscriber(jog+'/state',String,lambda m:store('backend_state',m.data),queue_size=50),
        rospy.Subscriber(tracker+'/run_start',Header,lambda m:store('run_id',str(m.stamp.to_nsec())),queue_size=10),
        rospy.Subscriber(tracker+'/run_metadata',String,lambda m:store('metadata',json.loads(m.data)),queue_size=10),
        rospy.Subscriber(tracker+'/reference_path',RosPath,reference,queue_size=5),
        rospy.Subscriber(tracker+'/samples',Float64MultiArray,lambda m:store('sample',list(m.data)),queue_size=50),
        rospy.Subscriber(initial['joint_states_topic'],JointState,feedback,queue_size=100,tcp_nodelay=True)]
    proxies={name:rospy.ServiceProxy(tracker+'/'+name,Trigger) for name in ['preview','start','stop']}
    disable=rospy.ServiceProxy(jog+'/set_enabled',SetBool)
    for name in proxies:
        rospy.wait_for_service(tracker+'/'+name, timeout=5)
    collector = None
    collector_log = None
    motion_owned = False
    initial_q = None
    first_origin = None
    first_orientation = None
    if previous is not None and previous['outcomes']:
        source=Path(previous['outcomes'][0]['source_folder'])/'metadata.json'
        old_frozen=json.loads(source.read_text())['frozen']
        first_origin=old_frozen['origin_m']
        first_orientation=old_frozen['start_orientation_xyzw']
    expected = None
    failure = None

    def service(name, timeout=15., request=None):
        result=queue.Queue(maxsize=1)
        def call():
            try:
                result.put((True,proxies[name]() if name in proxies else disable(request)))
            except Exception as exc:
                result.put((False,exc))
        threading.Thread(target=call,daemon=True).start()
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            try:
                ok,response=result.get(timeout=.05)
                if not ok:
                    raise RuntimeError(str(response))
                log('service',name=name,success=response.success,message=response.message)
                if not response.success:
                    raise RuntimeError(name+' rejected: '+response.message)
                return response
            except queue.Empty:
                if rospy.is_shutdown():
                    raise RuntimeError('Runner interrupted during '+name)
        raise RuntimeError('Service timeout/ambiguous outcome: '+name)

    def monitor():
        if rospy.is_shutdown():
            raise RuntimeError('Runner interrupted')
        state=read('state',.35)
        bottom=read('backend_state',.35)
        if state.startswith('FAULT:') or bottom.startswith('FAULT'):
            raise RuntimeError('Fault observed: '+state+' / '+bottom)
        f=read('feedback',.15)
        age=rospy.Time.now().to_sec()-f['stamp']
        if age < -.05 or age>.15:
            raise RuntimeError('Raw feedback timestamp stale/future: %.4f s'%age)
        if collector is not None and collector.poll() is not None:
            raise RuntimeError('Collector exited before the experiment completed')
        return state,bottom,f

    def wait_disabled(timeout=8.):
        until=time.monotonic()+timeout
        previous=None;stable_since=None
        while time.monotonic()<until:
            state,bottom,f=monitor()
            if bottom=='DISABLED' and not state.startswith(('ARMING:','RUNNING:','SETTLING:')):
                if previous and f['stamp']>previous['stamp']:
                    if f['names']!=previous['names']:
                        raise RuntimeError('Joint feedback order changed')
                    speed=max(abs(a-b)/(f['stamp']-previous['stamp']) for a,b in zip(f['q'],previous['q']))
                    if speed<=.003:
                        if stable_since is None:stable_since=time.monotonic()
                        if time.monotonic()-stable_since>=.7:return f
                    else:stable_since=None
                previous=f
            else:
                previous=None;stable_since=None
            time.sleep(.04)
        raise RuntimeError('DISABLED and stationary state not confirmed')

    def unchanged_profile(parameters):
        # Only these four tuning values are allowed to differ from initial.
        context={k:v for k,v in parameters.items() if k not in TUNING}
        baseline={k:v for k,v in initial.items() if k not in TUNING}
        if context!=baseline:
            raise RuntimeError('Trajectory/limits changed externally during the batch')
        if expected is not None:
            for k,v in expected.items():
                if abs(float(parameters[k])-v)>1e-8:
                    raise RuntimeError('Tuning parameters changed externally: '+k)

    try:
        # Initial subscription handshake. No service requests or tuning writes yet.
        until=time.monotonic()+5
        while time.monotonic()<until:
            with shared_lock:
                ready=all(k in shared for k in ['feedback','state','backend_state'])
            if ready:break
            time.sleep(.05)
        initial_q=wait_disabled()['q']
        master=rosgraph.Master(rospy.get_name())
        graph=master.getSystemState()
        pubs=next((nodes for topic,nodes in graph[0] if topic==jog+'/command'),[])
        subs=next((nodes for topic,nodes in graph[1] if topic==jog+'/command'),[])
        if pubs!=[tracker] or subs!=[jog_node]:
            raise RuntimeError('Execution topic topology changed: '+str((pubs,subs)))
        initial_model=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
        collector_log=(output/'capture_console.log').open('w')
        collector=subprocess.Popen([sys.executable,str(Path(__file__).with_name('tracking_experiment.py')),
            'record','--output',str(output/'capture'),'--label','actual_closed_loop_pid',
            '--only-new-runs'],stdout=collector_log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not list((output/'capture').glob('session_*/session.json')):
            monitor();time.sleep(.1)
        if not list((output/'capture').glob('session_*/session.json')):
            raise RuntimeError('Collector startup was not confirmed')
        time.sleep(.7)
        manifest['state']='RUNNING';write_json(output/'experiment.json',manifest)
        for index,case in enumerate(plan):
            f=wait_disabled()
            if max(abs(a-b) for a,b in zip(f['q'],initial_q))>.01:
                raise RuntimeError('Start joints drifted; no automatic repositioning is permitted')
            unchanged_profile(rospy.get_param(tracker))
            if hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()!=initial_model:
                raise RuntimeError('Robot model changed during the experiment')
            expected=dict(kp=float(initial['kp']),ki=case['ki'],kd=case['kd'],
                          acceleration_ff_time=float(initial['acceleration_ff_time']))
            for k,v in expected.items():rospy.set_param(tracker+'/'+k,v)
            unchanged_profile(rospy.get_param(tracker))
            log('trial_preparing',index=index+1,total=len(plan),**case,parameters=expected)
            old_reference=shared.get('reference',(None,{}))[1].get('stamp')
            service('preview',timeout=20.)
            until=time.monotonic()+3
            while time.monotonic()<until:
                current_reference=shared.get('reference',(None,{}))[1]
                if current_reference.get('stamp') is not None and current_reference['stamp']!=old_reference:
                    break
                monitor();time.sleep(.05)
            else:
                raise RuntimeError('Fresh preview reference was not received')
            origin=read('reference')['origin']
            orientation=read('reference')['orientation']
            if first_origin is None:first_origin=origin
            if first_orientation is None:first_orientation=orientation
            if math.sqrt(sum((a-b)**2 for a,b in zip(origin,first_origin)))>.002:
                raise RuntimeError('Start TCP drifted over 2mm; batch aborted without repositioning')
            dot=abs(sum(a*b for a,b in zip(orientation,first_orientation)))
            if 2*math.acos(min(1.,dot))>.02:
                raise RuntimeError('Start orientation drifted over 0.02rad')
            old_run=read('run_id')
            motion_owned=True
            service('start',timeout=5.)
            trial_started=time.monotonic();run_id=None;last_check=0.;last_progress=0.
            until=trial_started+float(initial['duration'])+float(initial['settle_timeout'])+8
            while time.monotonic()<until:
                state,bottom,f=monitor()
                current=read('run_id')
                if current not in ('0',old_run):
                    if run_id is None:
                        frozen=shared.get('metadata',(None,{}))[1]
                        if str(frozen.get('run_start_ns',''))!=current:
                            time.sleep(.02);continue
                        run_id=current
                        for k,v in expected.items():
                            if abs(float(frozen['parameters'][k])-v)>1e-8:
                                raise RuntimeError('Frozen parameters do not match scheduled trial')
                        log('trial_started',index=index+1,run_id=run_id,**case)
                if time.monotonic()-last_check>=1.:
                    unchanged_profile(rospy.get_param(tracker));last_check=time.monotonic()
                if run_id is not None:
                    values=shared.get('sample',(None,[]))[1]
                    if len(values)==16 and values[7]>.006:
                        raise RuntimeError('Experiment stop threshold exceeded: TCP error %.3f mm'%(values[7]*1000))
                    if time.monotonic()-last_progress>=10.:
                        log('progress',index=index+1,run_id=run_id,state=state,
                            elapsed_s=values[0] if values else None,error_mm=values[7]*1000 if values else None)
                        last_progress=time.monotonic()
                    if state.startswith('COMPLETED:'):
                        break
                    if state.startswith(('STOPPED:','IDLE:')):
                        raise RuntimeError('Trial stopped externally: '+state)
                elif time.monotonic()-trial_started>2.:
                    raise RuntimeError('Run did not enter RUNNING within two seconds')
                time.sleep(.04)
            else:
                raise RuntimeError('Trial exceeded its bounded runtime')
            wait_disabled();motion_owned=False
            deadline=time.monotonic()+6
            paths=[]
            while time.monotonic()<deadline:
                monitor()
                paths=list((output/'capture').glob('session_*/run_'+run_id+'/summary.json'))
                if paths:break
                time.sleep(.1)
            if len(paths)!=1:raise RuntimeError('Closed capture summary missing for '+str(run_id))
            summary=json.loads(paths[0].read_text())
            outcome=dict(case,run_id=run_id,source_folder=str(paths[0].parent),
                         all=summary['tcp']['all']['total_mm'],running=summary['tcp']['running']['total_mm'],
                         quality=summary['quality'],state=summary['state'])
            manifest['outcomes'].append(outcome);write_json(output/'experiment.json',manifest)
            log('trial_finished',index=index+1,**outcome)
            if summary['quality']:
                raise RuntimeError('Capture quality failure; further motion aborted: '+str(summary['quality']))
        manifest['state']='COMPLETE'
    except BaseException as exc:
        failure=str(exc) or type(exc).__name__
        manifest['state']='ABORTED';manifest['failure']=failure
        log('aborted',reason=failure)
    finally:
        if motion_owned:
            for attempt in range(2):
                try:service('stop',timeout=2.)
                except Exception as exc:log('stop_request_failed',error=str(exc))
                time.sleep(.3)
            try:service('disable',timeout=2.,request=False)
            except Exception as exc:log('disable_request_failed',error=str(exc))
            # A failed transport does not establish physical stopping. Do not reset/restart.
        for key in TUNING:
            try:rospy.set_param(tracker+'/'+key,initial[key])
            except Exception as exc:log('restore_failed',key=key,error=str(exc))
        if collector is not None:
            if collector.poll() is None:collector.send_signal(signal.SIGINT)
            try:collector.wait(timeout=15)
            except subprocess.TimeoutExpired:log('collector_exit_pending')
        if collector_log:collector_log.close()
        write_json(output/'experiment.json',manifest)
        for subscriber in subscribers:subscriber.unregister()
        if (output/'capture').exists():
            compare_runs([output/'capture'],output/'comparison')
        log('finished',state=manifest['state'],outcomes=len(manifest['outcomes']),output=str(output))
        journal.close();lock_file.close()
    return 0 if failure is None else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--execute',action='store_true',help='Explicitly request real preview/start/stop and tuning writes')
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--resume',help='After supervised recovery, run only uncompleted cases from this manifest')
    parser.add_argument('--backend',choices=['sdk','gazebo'],default='sdk')
    parser.add_argument('--tracker-ns',default='/trajectory_tracking')
    parser.add_argument('--jog-node-ns',default='/aubo_cartesian_jog')
    if argv is None:argv=[v for v in sys.argv[1:] if ':=' not in v]
    args=parser.parse_args(argv)
    if args.repeats<1 or args.repeats>3:parser.error('repeats must be 1..3')
    if not args.execute:
        plan=remaining_plan(load_resume_manifest(args.resume)) if args.resume else build_plan(args.repeats)
        print(json.dumps({'mode':'plan_only','plan':plan},indent=2))
        return 0
    return execute(args)


if __name__=='__main__':
    sys.exit(main())
