#!/usr/bin/env python3
"""Qt plot window for time-aligned measured/reference tracking samples."""
import csv
from collections import deque
import numpy as np
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog, QLabel, QCheckBox
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT


class TrackingPlot(QWidget):
    sample_received = Signal(object)
    headers = ['time_s', 'reference_x_m', 'reference_y_m', 'reference_z_m',
               'actual_x_m', 'actual_y_m', 'actual_z_m', 'error_m', 'actual_speed_m_s',
               'reference_speed_m_s', 'command_speed_m_s', 'trend_speed_m_s',
               'feedback_age_s', 'phase_settling', 'tangential_error_m', 'normal_error_m']

    def __init__(self):
        super().__init__()
        self.setWindowTitle('AUBO 跟踪曲线 · 目标 / 实际')
        self.resize(1100, 760)
        self.rows = deque(maxlen=200000)
        self.plane = 'xy'
        self.dirty = False
        self.figure = Figure(figsize=(11, 7), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.subplots(2, 2).ravel()
        layout = QVBoxLayout(self)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas)
        controls = QHBoxLayout()
        layout.addLayout(controls)
        for title, handler in [('保存 PNG', self.save_png), ('导出 CSV', self.save_csv)]:
            button = QPushButton(title)
            button.clicked.connect(handler)
            controls.addWidget(button)
        self.message = QLabel('等待跟踪数据；成功更新预览会清空上一轮数据。')
        self.relative = QCheckBox('相对起点坐标')
        self.relative.setChecked(True)
        self.relative.toggled.connect(self.coordinates_changed)
        controls.addWidget(self.relative)
        controls.addWidget(self.message)
        self.sample_received.connect(self.add_sample)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.redraw)
        self.timer.start(200)
        self.reset_axes()

    def reset_axes(self):
        for axis in self.axes:
            axis.clear()
            axis.grid(True, alpha=.25)
        first, second = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[self.plane]
        labels = 'XYZ'
        self.axes[0].set(xlabel=labels[first]+' (mm)', ylabel=labels[second]+' (mm)', title='TCP path: reference / actual')
        self.axes[0].set_aspect('equal', adjustable='datalim')
        self.path_lines = [self.axes[0].plot([], [], style, label=label)[0]
                           for style, label in [('c--', 'Reference'), ('C1-', 'Actual')]]
        self.axes[0].legend(loc='best')
        self.xyz_lines = []
        for i, label in enumerate(labels):
            for style, kind in [('--', 'ref'), ('-', 'actual')]:
                self.xyz_lines.append(self.axes[1].plot([], [], color='C'+str(i), linestyle=style, label=label+' '+kind)[0])
        self.axes[1].set(xlabel='Time (s)', ylabel='Position (mm)', title='XYZ displacement from start')
        self.axes[1].legend(ncol=3, fontsize=8)
        self.error_line, = self.axes[2].plot([], [], color='C3')
        self.along_line, = self.axes[2].plot([], [], '--', color='C0', alpha=.7, label='Tangential')
        self.normal_line, = self.axes[2].plot([], [], ':', color='C4', label='Normal')
        self.error_line.set_label('Total')
        self.axes[2].legend(loc='best', fontsize=8)
        self.axes[2].set(xlabel='Time (s)', ylabel='Error (mm)', title='TCP position error')
        self.speed_line, = self.axes[3].plot([], [], color='0.6', alpha=.5, linewidth=.8, label='Raw measured')
        self.trend_line, = self.axes[3].plot([], [], color='C2', label='Measured trend')
        self.ref_speed_line, = self.axes[3].plot([], [], 'c--', label='Reference')
        self.command_line, = self.axes[3].plot([], [], ':', color='C1', label='Command')
        self.axes[3].legend(loc='best', fontsize=8)
        self.axes[3].set(xlabel='Time (s)', ylabel='Speed (mm/s)', title='Measured TCP speed')
        self.settle_markers = []
        self.dirty = True

    def coordinates_changed(self, *_):
        self.dirty = True
        self.redraw()

    def set_plane(self, plane):
        if plane in ('xy', 'xz', 'yz') and plane != self.plane:
            self.plane = plane
            self.reset_axes()

    def add_sample(self, values):
        if not values:
            self.rows.clear()
            self.message.setText('新一轮预览成功，等待开始跟踪。')
        elif len(values) in (9, 16) and np.isfinite(values).all():
            # Accept old nodes; missing diagnostics remain explicitly unavailable.
            self.rows.append(tuple(values) + (float("nan"),)*(16-len(values)))
        self.dirty = True

    def redraw(self):
        if not self.dirty:
            return
        self.dirty = False
        if self.rows:
            # Render at most 5000 points; CSV keeps every received sample.
            data = np.asarray(self.rows)
            shown = data[::max(1, len(data)//5000)]
            offset = data[0, 1:4] if self.relative.isChecked() else np.zeros(3)
            a, b = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[self.plane]
            self.path_lines[0].set_data((shown[:, 1+a]-offset[a])*1000, (shown[:, 1+b]-offset[b])*1000)
            self.path_lines[1].set_data((shown[:, 4+a]-offset[a])*1000, (shown[:, 4+b]-offset[b])*1000)
            for i in range(3):
                self.xyz_lines[2*i].set_data(shown[:, 0], (shown[:, 1+i]-offset[i])*1000)
                self.xyz_lines[2*i+1].set_data(shown[:, 0], (shown[:, 4+i]-offset[i])*1000)
            self.error_line.set_data(shown[:, 0], shown[:, 7]*1000)
            self.speed_line.set_data(shown[:, 0], shown[:, 8]*1000)
            self.trend_line.set_data(shown[:, 0], shown[:, 11]*1000)
            self.ref_speed_line.set_data(shown[:, 0], shown[:, 9]*1000)
            self.command_line.set_data(shown[:, 0], shown[:, 10]*1000)
            self.along_line.set_data(shown[:, 0], shown[:, 14]*1000)
            self.normal_line.set_data(shown[:, 0], shown[:, 15]*1000)
            for marker in self.settle_markers:
                marker.remove()
            self.settle_markers = []
            settling = data[data[:, 13] == 1]
            if len(settling):
                for axis in self.axes[1:]:
                    self.settle_markers.append(axis.axvline(settling[0, 0], color='0.5', linestyle='--', alpha=.6))
            self.axes[1].set_title('XYZ displacement from start' if self.relative.isChecked() else 'XYZ position')
            self.message.setText('%d 点 | 峰值 %.2f mm | RMS %.2f mm | 末点 %.2f mm' %
                                 (len(data), np.max(data[:, 7])*1000, np.sqrt(np.mean(data[:, 7]**2))*1000, data[-1, 7]*1000))
        else:
            for marker in self.settle_markers:
                marker.remove()
            self.settle_markers = []
            for axis in self.axes:
                for line in axis.lines:
                    line.set_data([], [])
        for axis in self.axes:
            axis.relim()
            axis.autoscale_view()
        self.canvas.draw_idle()

    def save_png(self):
        path, _ = QFileDialog.getSaveFileName(self, '保存跟踪曲线', 'tracking.png', 'PNG (*.png)')
        if path:
            self.redraw()
            self.figure.savefig(path, dpi=160)
            self.message.setText('已保存 '+path)

    def save_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出跟踪数据', 'tracking.csv', 'CSV (*.csv)')
        if path:
            with open(path, 'w', newline='') as output:
                writer = csv.writer(output)
                writer.writerow(self.headers)
                writer.writerows(self.rows)
            self.message.setText('已导出 '+path)

    def closeEvent(self, event):
        # Keep collecting while hidden, so opening the window later shows this run.
        self.hide()
        event.ignore()
