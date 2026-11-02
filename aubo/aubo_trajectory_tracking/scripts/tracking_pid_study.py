#!/usr/bin/env python3
"""Offline I/D signal screening on a fixed observed trajectory, not a plant simulator."""
import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np

from tracking_experiment_data import read_rows, stats, write_json


def limited(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector*(limit/norm) if norm > limit else vector.copy()


def curve_samples(times, parameters, origin):
    """Same analytic quintic reference as tracking_curve.h, at control timestamps."""
    p = parameters
    u = np.clip(np.asarray(times)/float(p['duration']), 0., 1.)
    phase = u**3*(10.+u*(-15.+6.*u))
    phase_v = 30.*u*u*(1.-u)**2/float(p['duration'])
    phase_a = 60.*u*(1.-u)*(1.-2.*u)/float(p['duration'])**2
    zero = np.zeros_like(u)
    if p['shape'] == 'circle':
        theta = 2*np.pi*float(p['cycles'])*phase
        omega = 2*np.pi*float(p['cycles'])*phase_v
        alpha = 2*np.pi*float(p['cycles'])*phase_a
        r = float(p['radius'])
        xy = [r*(np.cos(theta)-1), r*np.sin(theta)]
        velocity = [-r*np.sin(theta)*omega, r*np.cos(theta)*omega]
        acceleration = [-r*(np.cos(theta)*omega**2+np.sin(theta)*alpha),
                        r*(-np.sin(theta)*omega**2+np.cos(theta)*alpha)]
    elif p['shape'] == 'sine':
        theta = 2*np.pi*float(p['waves'])*phase
        omega = 2*np.pi*float(p['waves'])*phase_v
        alpha = 2*np.pi*float(p['waves'])*phase_a
        length, amplitude = float(p['length']), float(p['amplitude'])
        xy = [length*phase, amplitude*np.sin(theta)]
        velocity = [length*phase_v, amplitude*np.cos(theta)*omega]
        acceleration = [length*phase_a, amplitude*(-np.sin(theta)*omega**2+np.cos(theta)*alpha)]
    else:
        raise ValueError('Unsupported reference shape')
    a, b = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[p['plane']]
    def expand(pair):
        result = np.column_stack([zero, zero, zero])
        result[:, a], result[:, b] = pair
        return result
    return expand(xy)+np.asarray(origin), expand(velocity), expand(acceleration)


def velocity_window(times, positions, window):
    history = deque()
    result = []
    for t, position in zip(times, positions):
        if history and (t <= history[-1][0] or t-history[-1][0] > window*2):
            history.clear()
        history.append((t, position))
        while len(history)>2 and t-history[0][0]>window:
            history.popleft()
        x = np.asarray([row[0]-t for row in history])
        y = np.asarray([row[1] for row in history])
        x = x-x.mean()
        variance = float(x.dot(x))
        result.append(x.dot(y-y.mean(axis=0))/variance if variance>1e-12 else np.zeros(3))
    return np.asarray(result)


def replay(times, feedforward, error, velocity_error, kp, ki, kd, speed, fraction, tolerance, duration):
    """Reconstruct TrackingCorrection outputs with observed inputs held fixed.

    Returns command signals only. The actual error trace is unchanged; these
    are not predictions of tracking error or closed-loop stability.
    """
    times = np.asarray(times)
    positive = np.diff(times)
    if len(times)<2 or not np.isfinite(times).all() or np.any(positive<=0):
        raise ValueError('Control times must be finite and strictly increasing')
    period = float(np.median(positive))
    first_dt = float(times[0]) if 0<times[0]<.1 else period
    dts = np.r_[first_dt, positive]
    budget = speed*fraction
    integral = np.zeros(3)
    holding = False
    command, integrals, derivatives, raw_correction, corrections, antiwindup = [], [], [], [], [], []
    for t, dt, ff, e, ve in zip(times, dts, feedforward, error, velocity_error):
        d = kd*ve
        blocked = False
        if t >= duration:
            integral = integral*math.exp(-dt/.5)
            if np.linalg.norm(e) <= .5*tolerance:
                holding = True
            elif np.linalg.norm(e) > tolerance:
                holding = False
            if holding:
                integral = np.zeros(3)
                integrals.append(integral.copy()); derivatives.append(np.zeros(3))
                raw_correction.append(np.zeros(3)); corrections.append(np.zeros(3))
                command.append(np.zeros(3)); antiwindup.append(False)
                continue
        else:
            candidate = limited(integral+e*(ki*dt), budget)
            raw = kp*e+candidate+d
            saturating = np.linalg.norm(raw)>budget or np.linalg.norm(ff+raw)>speed
            if not saturating or float(np.dot(e, integral))<0:
                integral = candidate
            else:
                blocked = ki>0
        raw = kp*e+integral+d
        correction = limited(raw, budget)
        command.append(limited(ff+correction, speed))
        integrals.append(integral.copy()); derivatives.append(d.copy())
        raw_correction.append(raw.copy()); corrections.append(correction.copy()); antiwindup.append(blocked)
    return {name: np.asarray(values) for name, values in [
        ('command', command), ('integral', integrals), ('derivative', derivatives),
        ('raw_correction', raw_correction), ('correction', corrections), ('antiwindup', antiwindup)]}


def analyze_pid_inputs(folder, ki_grid, kd_grid, output):
    folder, output = Path(folder), Path(output)
    meta = json.loads((folder/'metadata.json').read_text())
    frozen = meta.get('frozen')
    if not frozen or 'origin_m' not in frozen:
        raise ValueError('Requires frozen parameters and preview origin')
    p = frozen['parameters']
    rows = np.asarray(read_rows(folder, 'tcp'))
    if len(rows)<3:
        raise ValueError('Requires at least three valid TCP samples')
    t, measured = rows[:, 0], rows[:, 4:7]
    control_t = t+rows[:, 12]
    ref_now, ref_velocity, ref_acceleration = curve_samples(control_t, p, frozen['origin_m'])
    # The production estimator compares absolute toSec() doubles at an 80ms
    # boundary. Preserve that rounding: relative time alone can choose a
    # different window at exact multiples of 20ms and fabricate D spikes.
    stamp_ns = np.int64(int(meta['run_id']))+np.rint(t*1e9).astype(np.int64)
    absolute_t = (stamp_ns//1000000000).astype(float)+(stamp_ns%1000000000).astype(float)*1e-9
    trend = velocity_window(absolute_t, measured, float(p.get('velocity_window', .08)))
    long_trend = velocity_window(absolute_t, measured, .20)
    error_control = ref_now-measured
    error_aligned = rows[:, 1:4]-measured
    velocity_error = ref_velocity-trend
    speed, fraction = float(p['max_speed']), float(p['correction_fraction'])
    ff = np.asarray([limited(v+limited(a*float(p['acceleration_ff_time']), .1*speed), .8*speed)
                     for v,a in zip(ref_velocity, ref_acceleration)])
    running = (rows[:, 13] == 0)
    active = control_t<float(p['duration'])
    if np.count_nonzero(running)<3 or np.count_nonzero(active)<3:
        raise ValueError('Insufficient running samples')
    baseline = replay(control_t, ff, error_control, velocity_error, float(p['kp']),
                      float(p['ki']), float(p['kd']), speed, fraction,
                      float(p['settle_tolerance']), float(p['duration']))
    baseline_error = np.linalg.norm(baseline['command'], axis=1)-rows[:, 10]
    speed_reconstruction_error = np.linalg.norm(trend, axis=1)-rows[:, 11]
    norm_error = np.linalg.norm(error_aligned, axis=1)
    rt, re = t[running], error_aligned[running]
    bias = re.mean(axis=0)
    area = np.trapz(re, rt, axis=0)
    # Spectral values describe the changing trajectory error, not a stationary
    # vibration/stability measurement. Interpolate timestamps before the FFT.
    step = float(np.median(np.diff(rt)))
    uniform_t = np.arange(rt[0], rt[-1], step)
    uniform_error = np.column_stack([np.interp(uniform_t, rt, re[:, j]) for j in range(3)])
    uniform_error -= uniform_error.mean(axis=0)
    power = np.sum(np.abs(np.fft.rfft(uniform_error*np.hanning(len(uniform_t))[:, None], axis=0))**2, axis=1)
    frequencies = np.fft.rfftfreq(len(uniform_t), step)
    non_dc = frequencies>0
    total_power = float(power[non_dc].sum())
    diagnostics = {
        'run_id': meta['run_id'], 'source_folder': str(folder.resolve()), 'state': meta.get('state'),
        'frozen_parameters': p, 'sample_count': len(rows), 'running_samples': int(running.sum()),
        'observed_running_error_mm': stats(norm_error[running], 1000),
        'mean_xyz_error_mm': (bias*1000).tolist(), 'mean_error_vector_norm_mm': float(np.linalg.norm(bias)*1000),
        'signed_error_area_xyz_mm_s': (area*1000).tolist(),
        'iae_mm_s': float(np.trapz(norm_error[running]*1000, rt)),
        'ise_mm2_s': float(np.trapz((norm_error[running]*1000)**2, rt)),
        'itae_mm_s2': float(np.trapz(rt*norm_error[running]*1000, rt)),
        'fraction_over_1mm': float(np.mean(norm_error[running]>.001)),
        'fraction_over_2mm': float(np.mean(norm_error[running]>.002)),
        'velocity_error_mm_s': stats(np.linalg.norm(velocity_error[active], axis=1), 1000),
        'velocity_error_peak_time_s': float(control_t[active][np.argmax(np.linalg.norm(velocity_error[active], axis=1))]),
        'baseline_command_norm_reconstruction_error_mm_s': stats(baseline_error[active], 1000),
        'trend_speed_reconstruction_error_mm_s': stats(speed_reconstruction_error[active], 1000),
        'dominant_error_frequency_hz': float(frequencies[non_dc][np.argmax(power[non_dc])]) if total_power>0 else None,
        'error_spectral_power_above_1hz_fraction': float(power[frequencies>1].sum()/total_power) if total_power>0 else None,
        'correction_budget_mm_s': speed*fraction*1000,
        'warnings': ['Signal replay holds the observed trajectory fixed; no candidate tracking RMS, peak, stability or optimality is predicted.',
                     'Velocity is reconstructed from received TCP samples; callbacks without a published sample and pre-run window history are unavailable.',
                     'Spectral components include the nonstationary reference maneuver and cannot alone diagnose mechanical vibration.']}
    # Retain acquisition quality flags as context rather than silently treating
    # a truncated/faulted run as a successful I/D experiment.
    if (folder/'summary.json').exists():
        diagnostics['capture_quality'] = json.loads((folder/'summary.json').read_text()).get('quality', [])
    if float(np.max(np.diff(control_t))) > .1:
        diagnostics['warnings'].append('Control reconstruction contains gaps over 100ms; integral history is approximate.')
    diagnostics['stages'] = []
    for name, lo, hi in [('start',0,.125),('accelerating',.125,.375),('middle',.375,.625),
                         ('decelerating',.625,.875),('end',.875,1.)]:
        mask=running & (t>=lo*float(p['duration'])) & (t<hi*float(p['duration']))
        diagnostics['stages'].append(dict(name=name, start_s=lo*float(p['duration']),
            end_s=hi*float(p['duration']), error_mm=stats(norm_error[mask],1000),
            mean_xyz_error_mm=error_aligned[mask].mean(axis=0).tolist() if mask.any() else None))
        if mask.any():
            diagnostics['stages'][-1]['mean_xyz_error_mm']=(error_aligned[mask].mean(axis=0)*1000).tolist()
    candidates = []
    for ki in ki_grid:
        for kd in kd_grid:
            result = replay(control_t, ff, error_control, velocity_error, float(p['kp']), ki, kd,
                            speed, fraction, float(p['settle_tolerance']), float(p['duration']))
            def magnitude(key):
                return np.linalg.norm(result[key][active], axis=1)*1000
            d_steps = np.linalg.norm(np.diff(result['derivative'][active], axis=0), axis=1)*1000
            window_sensitivity = kd*np.linalg.norm(trend[active]-long_trend[active], axis=1)*1000
            candidates.append(dict(ki=ki, kd=kd,
                i_peak_mm_s=float(magnitude('integral').max()),
                i_rms_mm_s=float(np.sqrt(np.mean(magnitude('integral')**2))),
                d_peak_mm_s=float(magnitude('derivative').max()),
                d_rms_mm_s=float(np.sqrt(np.mean(magnitude('derivative')**2))),
                d_step_p95_mm_s=float(np.percentile(d_steps,95)),
                d_window_sensitivity_rms_mm_s=float(np.sqrt(np.mean(window_sensitivity**2))),
                correction_peak_mm_s=float(magnitude('correction').max()),
                correction_budget_peak_fraction=float(magnitude('raw_correction').max()/(speed*fraction*1000)),
                correction_saturation_fraction=float(np.mean(magnitude('raw_correction')>speed*fraction*1000+1e-9)),
                antiwindup_blocked_fraction=float(np.mean(result['antiwindup'][active])),
                command_peak_mm_s=float(magnitude('command').max()),
                command_change_rms_mm_s=float(np.sqrt(np.mean(np.sum((
                    result['command'][active]-baseline['command'][active])**2, axis=1)))*1000)))
    output.mkdir(parents=True, exist_ok=True)
    write_json(output/'id_statistics.json', diagnostics)
    with (output/'id_candidates.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(candidates[0])); writer.writeheader(); writer.writerows(candidates)
    # These are small, separate experiment candidates, not an optimized or
    # automatically applied PID. Preserve P, feedforward, geometry and limits.
    for label, ki, kd in [('baseline',float(p['ki']),float(p['kd'])), ('I002',.02,0.),
                           ('I005',.05,0.), ('D001',0.,.01), ('D002',0.,.02), ('I002_D002',.02,.02)]:
        candidate = dict(p, ki=ki, kd=kd)
        for name in ('waves', 'cycles'):
            if name in candidate:
                candidate[name] = int(candidate[name])
        candidate.update({k:frozen[k] for k in ('base_link','tool_link') if k in frozen})
        write_json(output/('candidate_'+label+'.yaml'), candidate)
    # Save diagnostic inputs so the command calculation is independently auditable.
    matrix=np.column_stack([control_t, ff, error_control, velocity_error])
    np.savetxt(output/'replay_inputs.csv',matrix,delimiter=',',
               header='time_s,ff_x_m_s,ff_y_m_s,ff_z_m_s,error_x_m,error_y_m,error_z_m,velocity_error_x_m_s,velocity_error_y_m_s,velocity_error_z_m_s',comments='')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for j,label in enumerate('XYZ'):
        axes[0,0].plot(t,error_aligned[:,j]*1000,label=label)
        axes[0,1].plot(t,velocity_error[:,j]*1000,label=label)
    axes[0,0].set(title='Signed Cartesian error',xlabel='Run time (s)',ylabel='mm')
    axes[0,1].set(title='Reference minus reconstructed trend velocity',xlabel='Run time (s)',ylabel='mm/s')
    for ki,kd in [(.02,0.),(.05,0.),(0.,.01),(0.,.02)]:
        result=replay(control_t,ff,error_control,velocity_error,float(p['kp']),ki,kd,
                      speed,fraction,float(p['settle_tolerance']),float(p['duration']))
        key='integral' if ki else 'derivative'
        axis=axes[1,0] if ki else axes[1,1]
        axis.plot(t,np.linalg.norm(result[key],axis=1)*1000,label=('Ki=%g'%ki if ki else 'Kd=%g'%kd))
    axes[1,0].set(title='I term on fixed observed inputs (not error prediction)',xlabel='Run time (s)',ylabel='Correction (mm/s)')
    axes[1,1].set(title='D term on fixed observed inputs (not error prediction)',xlabel='Run time (s)',ylabel='Correction (mm/s)')
    for axis in axes.flat:
        axis.grid(alpha=.3);axis.legend(fontsize=8)
    fig.savefig(output/'id_diagnostics.png',dpi=150);plt.close(fig)
    lines=['# I/D 离线分析与候选修正量', '',
           '**以下没有执行新 I/D 实机实验，也不预测新参数的轨迹峰值、RMS 或闭环稳定性。**',
           '回放保留实际轨迹不变，只重算候选控制输出与限幅占用；不能用输出变小或不饱和证明整定有效。', '',
           '误差统计对齐到反馈时刻；控制回放用“反馈时刻 + 反馈年龄”重建当前参考，速度向量按冻结窗口做时间戳回归。',
           'I=D=0 的基线模长吻合只验证参考/P/前馈重建，不证明 D 的三维速度输入完全相同；未发布周期和运行前窗口历史未知。', '',
           '| 实测/重建指标 | 数值 |','|---|---:|',
           '| 运行段实测 RMS | %.4f mm |'%diagnostics['observed_running_error_mm']['rms'],
           '| 平均 XYZ 误差向量模长 | %.4f mm |'%diagnostics['mean_error_vector_norm_mm'],
           '| 重建速度误差 RMS / 峰值 | %.3f / %.3f mm/s |'%(diagnostics['velocity_error_mm_s']['rms'],diagnostics['velocity_error_mm_s']['peak_abs']),
           '| 基线指令速度模长重建偏差 RMS | %.6f mm/s |'%diagnostics['baseline_command_norm_reconstruction_error_mm_s']['rms'],
           '| 趋势速度模长重建偏差 RMS | %.4f mm/s |'%diagnostics['trend_speed_reconstruction_error_mm_s']['rms'],
           '| IAE | %.4f mm·s |'%diagnostics['iae_mm_s'],
           '| ISE | %.4f mm²·s |'%diagnostics['ise_mm2_s'],
           '| ITAE | %.4f mm·s² |'%diagnostics['itae_mm_s2'],
           '| 大于 1 mm / 2 mm 的采样占比 | %.2f%% / %.2f%% |'%(100*diagnostics['fraction_over_1mm'],100*diagnostics['fraction_over_2mm']), '',
           '平均 XYZ 偏差不是径向偏差的均值；当前积分器分别积累有符号 XYZ 误差，不能对误差模长积分。',
           '末点短时收敛不能替代所有姿态下的静态负载测试。频谱包含变化的参考运动，不直接判定振荡或稳定裕量。', '',
           '## 分阶段统计', '', '| 阶段 | 时间 s | RMS mm | 峰值 mm |','|---|---|---:|---:|']
    for stage in diagnostics['stages']:
        value=stage['error_mm']
        if value['n']:
            lines.append('| %s | %.1f–%.1f | %.4f | %.4f |'%(stage['name'],stage['start_s'],stage['end_s'],value['rms'],value['peak_abs']))
    lines += ['', '## 候选项信号筛查', '',
              '| Ki | Kd | I 峰值 mm/s | D 峰值 mm/s | 纠偏预算峰值占比 | 饱和采样占比 |',
              '|---:|---:|---:|---:|---:|---:|']
    for c in candidates:
        lines.append('| %.3f | %.3f | %.3f | %.3f | %.1f%% | %.2f%% |'%(
            c['ki'],c['kd'],c['i_peak_mm_s'],c['d_peak_mm_s'],100*c['correction_budget_peak_fraction'],100*c['correction_saturation_fraction']))
    lines += ['', '先保留当前 P 与前馈。已生成分别只加 I=0.02、I=0.05、D=0.01、D=0.02 的候选文件，',
              '以及 I=0.02 / D=0.02 组合候选；组合应在单项对照之后评估。候选未自动应用，',
              '其实际效果须由新的同工况闭环记录验证，不能把本次信号回放当成已测的 I/D 结果。', '',
              '![I/D 输入与修正量](id_diagnostics.png)', '', '## 限制', '']+diagnostics['warnings']
    (output/'id_report.md').write_text('\n'.join(lines)+'\n')
    return {'run_id':meta['run_id'],'folder':str(output.resolve()),'candidates':len(candidates),
            'baseline_reconstruction_rms_mm_s':diagnostics['baseline_command_norm_reconstruction_error_mm_s']['rms']}


def study_runs(inputs, output, ki_grid, kd_grid):
    folders=set()
    for source in inputs:
        source=Path(source)
        if (source/'metadata.json').exists():folders.add(source.resolve())
        else:folders.update(p.parent.resolve() for p in source.rglob('metadata.json'))
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    completed=[];issues=[]
    for folder in sorted(folders):
        try:
            meta=json.loads((folder/'metadata.json').read_text())
            if meta.get('state')=='OPEN':
                issues.append(str(folder)+': still open');continue
            target=output/(folder.parent.name+'_'+folder.name)
            completed.append(analyze_pid_inputs(folder,ki_grid,kd_grid,target))
        except (ValueError,KeyError,OSError,TypeError) as exc:
            issues.append(str(folder)+': '+str(exc))
    write_json(output/'id_study.json',{'runs':completed,'issues':issues,
        'scope':'Offline signal replay only; no candidate closed-loop errors are predicted.'})
    lines=['# I/D 批量离线分析','', '未改变真机参数；每组仅比较固定实测输入下的修正量，不预测新轨迹误差。','']
    for run in completed:
        path=Path(run['folder'])
        lines.append('- ['+run['run_id']+']('+str(path.relative_to(output.resolve()))+'/id_report.md)：'+str(run['candidates'])+' 组候选')
    lines+=['','## 未处理记录','']+issues
    (output/'I_D批量分析.md').write_text('\n'.join(lines)+'\n')
    return {'runs':completed,'issues':issues}
