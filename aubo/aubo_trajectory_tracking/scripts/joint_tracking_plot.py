#!/usr/bin/env python3
"""Six joint command/measurement plots and servo-chain diagnostics, SI CSV."""
import csv
import os
from collections import deque
import numpy as np
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QLabel, QFileDialog
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT


class JointTrackingPlot(QWidget):
    sample_received = Signal(object)
    servo_received = Signal(object)
    status_received = Signal(str)
    joint_names = ['Shoulder', 'Upper arm', 'Forearm', 'Wrist 1', 'Wrist 2', 'Wrist 3']
    headers = ['time_s'] + [kind+'_j'+str(j+1)+unit for kind, unit in
              [('target', '_rad'), ('actual', '_rad'), ('error', '_rad'),
               ('target_velocity', '_rad_s'), ('actual_velocity', '_rad_s')] for j in range(6)] + [
              'target_interpolation_gap_s', 'backend_0_gazebo_1_ros_control_2_sdk', 'velocity_source_1_message_0_difference']
    servo_headers = ['time_s'] + [kind+'_'+axis+'_m_s' for kind in ('requested', 'resolved', 'smoothed') for axis in 'xyz'] + [
                     'sigma_min', 'dls_lambda', 'velocity_scale', 'backend_0_gazebo_1_ros_control_2_sdk']

    def __init__(self):
        super().__init__()
        self.rows = deque(maxlen=200000)
        self.servo_rows = deque(maxlen=200000)
        self.tabs = QTabWidget()
        self.status = QLabel('等待六轴关节目标和实测数据；使用关节角反馈。')
        self.status.setWordWrap(True)
        self.summary = QLabel('')
        self.summary.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.tabs)
        layout.addWidget(self.summary)
        self.figures, self.canvases, self.axes, self.lines = [], [], [], []
        for kind, title in [('position', '六轴位置'), ('error', '六轴误差'), ('velocity', '六轴速度'), ('servo', '控制链路诊断')]:
            figure = Figure(figsize=(12, 8), tight_layout=True)
            canvas = FigureCanvasQTAgg(figure)
            axes = figure.subplots(2, 2).ravel() if kind == 'servo' else figure.subplots(3, 2).ravel()
            page = QWidget()
            page_layout = QVBoxLayout(page)
            page_layout.addWidget(NavigationToolbar2QT(canvas, page))
            page_layout.addWidget(canvas)
            lines = []
            if kind != 'servo':
                for j, axis in enumerate(axes):
                    axis.set_title('J%d - %s' % (j+1, self.joint_names[j]))
                    axis.set_xlabel('Time (s)')
                    axis.grid(True, alpha=.25)
                    axis.set_ylabel('Error (deg)' if kind == 'error' else 'Velocity (deg/s)' if kind == 'velocity' else 'Position (deg)')
                    if kind == 'error':
                        lines.append([axis.plot([], [], color='C3', label='Target - measured')[0]])
                    else:
                        lines.append([axis.plot([], [], 'c--', label='Generated target')[0],
                                      axis.plot([], [], color='C1', label='Measured')[0]])
                    axis.legend(fontsize=8, loc='best')
            else:
                for axis in axes:
                    axis.grid(True, alpha=.25)
                    axis.set_xlabel('Time (s)')
                for color, label in [('C0', 'Requested'), ('C1', 'After IK / limits'), ('C2', 'After smoothing')]:
                    lines.append(axes[0].plot([], [], color=color, label=label)[0])
                axes[0].set(ylabel='Speed (mm/s)', title='Model Cartesian speed through servo chain')
                lines.append(axes[1].plot([], [], label='Min singular value')[0])
                lines.append(axes[1].plot([], [], label='DLS damping')[0])
                axes[1].set(title='Kinematic conditioning / damping')
                lines.append(axes[2].plot([], [], color='C3', label='Limit scale')[0])
                axes[2].set(ylabel='Scale', title='Singularity / joint limit speed scaling', ylim=(-.05, 1.05))
                lines.append(axes[3].plot([], [], color='C4', label='Requested - resolved')[0])
                axes[3].set(ylabel='Vector residual (mm/s)', title='IK / limit model residual')
                for axis in axes: axis.legend(fontsize=8, loc='best')
            self.tabs.addTab(page, title)
            self.figures.append(figure); self.canvases.append(canvas)
            self.axes.append(axes); self.lines.append(lines)
        self.dirty = [True]*4
        self.sample_received.connect(self.add_sample)
        self.servo_received.connect(self.add_servo)
        self.status_received.connect(self.status.setText)
        self.tabs.currentChanged.connect(lambda _: self.redraw())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.redraw)
        self.timer.start(200)

    def add_sample(self, values):
        if not values:
            self.rows.clear()
        elif len(values) == 34 and np.isfinite(values).all():
            self.rows.append(tuple(values))
        self.dirty[:3] = [True]*3

    def add_servo(self, values):
        if not values:
            self.servo_rows.clear()
        elif len(values) == 14 and np.isfinite(values).all():
            self.servo_rows.append(tuple(values))
        self.dirty[3] = True

    def render(self, view):
        if not self.dirty[view]: return
        self.dirty[view] = False
        if view < 3 and self.rows:
            data = np.asarray(self.rows)
            indexes = np.arange(0, len(data), max(1, len(data)//5000))
            gaps = np.nonzero(np.diff(data[:, 0]) > .15)[0]+1
            indexes = np.unique(np.concatenate([indexes, gaps]))
            shown = data[indexes].copy()
            shown[np.isin(indexes, gaps), 1:31] = np.nan
            for j in range(6):
                if view == 0:
                    self.lines[view][j][0].set_data(shown[:, 0], np.rad2deg(shown[:, 1+j]))
                    self.lines[view][j][1].set_data(shown[:, 0], np.rad2deg(shown[:, 7+j]))
                elif view == 1:
                    self.lines[view][j][0].set_data(shown[:, 0], np.rad2deg(shown[:, 13+j]))
                else:
                    self.lines[view][j][0].set_data(shown[:, 0], np.rad2deg(shown[:, 19+j]))
                    self.lines[view][j][1].set_data(shown[:, 0], np.rad2deg(shown[:, 25+j]))
            rms = np.rad2deg(np.sqrt(np.mean(data[:, 13:19]**2, axis=0)))
            peak = np.rad2deg(np.max(np.abs(data[:, 13:19]), axis=0))
            suffix = 'SDK 目标是供点生成位置，含队列超前，不能直接视作电机伺服误差。' if data[-1, 32] == 2 else '已按反馈时间戳对齐生成指令目标。'
            self.summary.setText('J1–J6 RMS (deg): '+', '.join('%.3f' % x for x in rms)+'\n峰值 (deg): '+', '.join('%.3f' % x for x in peak)+'\n'+suffix)
        elif view == 3 and self.servo_rows:
            data = np.asarray(self.servo_rows)
            shown = data[::max(1, len(data)//5000)]
            for i in range(3):
                self.lines[3][i].set_data(shown[:, 0], np.linalg.norm(shown[:, 1+3*i:4+3*i], axis=1)*1000)
            self.lines[3][3].set_data(shown[:, 0], shown[:, 10])
            self.lines[3][4].set_data(shown[:, 0], shown[:, 11])
            self.lines[3][5].set_data(shown[:, 0], shown[:, 12])
            self.lines[3][6].set_data(shown[:, 0], np.linalg.norm(shown[:, 1:4]-shown[:, 4:7], axis=1)*1000)
        else:
            for axis in self.axes[view]:
                for line in axis.lines: line.set_data([], [])
            if view < 3: self.summary.setText('等待本轮六轴数据；未提供目标时不绘制虚构参考。')
        for i, axis in enumerate(self.axes[view]):
            axis.relim()
            if view == 3 and i == 2: axis.autoscale_view(scaley=False)
            else: axis.autoscale_view()
        self.canvases[view].draw_idle()

    def redraw(self):
        if self.isVisible(): self.render(self.tabs.currentIndex())

    def save_png(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出六轴位置/误差/速度及链路图', 'joint_tracking.png', 'PNG (*.png)')
        if not path: return
        root, _ = os.path.splitext(path)
        for view, suffix in enumerate(('positions', 'errors', 'velocities', 'servo')):
            self.render(view)
            self.figures[view].savefig(root+'_'+suffix+'.png', dpi=160)
        self.status.setText('已导出 '+root+'_{positions,errors,velocities,servo}.png')

    def save_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出关节与链路原始数据 (rad, rad/s)', 'joint_tracking.csv', 'CSV (*.csv)')
        if not path: return
        root, _ = os.path.splitext(path)
        for name, headers, rows in [(path, self.headers, self.rows), (root+'_servo.csv', self.servo_headers, self.servo_rows)]:
            with open(name, 'w', newline='') as output:
                writer = csv.writer(output); writer.writerow(headers); writer.writerows(rows)
        self.status.setText('已导出 '+path+' 和 '+root+'_servo.csv')
