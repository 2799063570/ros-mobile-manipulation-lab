#!/usr/bin/env python3
"""Small Qt client; ROS service calls run off the GUI thread so Stop stays responsive."""
import sys
import os
import threading
import time
import rospy
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from python_qt_binding.QtCore import QObject, Signal, QTimer
from python_qt_binding.QtWidgets import (QApplication, QWidget, QVBoxLayout, QFormLayout,
                                        QHBoxLayout, QComboBox, QDoubleSpinBox, QSpinBox,
                                        QPushButton, QLabel)


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tracking_plot import TrackingPlot


class Events(QObject):
    state = Signal(str)
    result = Signal(str)
    metrics = Signal(str)
    ready = Signal()
    readiness = Signal(str)
    previewed = Signal()
    closed = Signal()
    preparation = Signal(str)
    analyzed = Signal()


class Panel(QWidget):
    def __init__(self):
        super().__init__()
        self.ns = rospy.get_param('~tracker_ns', '/trajectory_tracking')
        self.events = Events()
        self.prep_ns = "/trajectory_preparation"
        self.preparation_kind = "IDLE"
        self.preparation_time = 0.
        self.preparation_dirty = True
        self.plot = TrackingPlot()
        self.close_pending = False
        self.close_ready = False
        self.busy = False
        self.params_dirty = False
        self.readiness_kind = "WAITING"
        self.readiness_time = 0.
        self.setWindowTitle('AUBO 轨迹跟踪')
        self.resize(600, 780)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('当前 TCP 为起点 · 基坐标系 · 固定末端姿态'))
        form = QFormLayout()
        layout.addLayout(form)
        self.fields = {}
        for key, title, choices in [('shape', '轨迹', ['circle', 'sine']), ('plane', '平面', ['xy', 'xz', 'yz'])]:
            box = QComboBox()
            box.addItems(choices)
            box.setCurrentText(rospy.get_param(self.ns + '/' + key, choices[0]))
            self.fields[key] = box
            form.addRow(title, box)
        for key, title, lo, hi, default in [
            ('radius', '圆半径 (m)', .001, .20, .03),
            ('amplitude', '正弦振幅 (m)', .001, .20, .02),
            ('length', '正弦前进距离 (m)', .001, .40, .10),
            ('duration', '总时长 (s)', 2., 3600., 40.),
            ('max_speed', '速度上限 (m/s)', .001, .10, .015),
            ('max_extent', '允许最大位移 (m)', .01, .50, .25),
            ('kp', '位置纠偏增益 (1/s)', .05, 3., 1.),
            ('ki', '积分增益 (默认关闭)', 0., 2., 0.)]:
            box = QDoubleSpinBox()
            box.setDecimals(3)
            box.setRange(lo, hi)
            box.setSingleStep(.001 if key != 'duration' else 5.)
            box.setValue(rospy.get_param(self.ns + '/' + key, default))
            self.fields[key] = box
            form.addRow(title, box)
        for key, title in [('cycles', '圆圈数'), ('waves', '正弦波数')]:
            box = QSpinBox()
            box.setRange(1, 20)
            box.setValue(rospy.get_param(self.ns + '/' + key, 1))
            self.fields[key] = box
            form.addRow(title, box)
        preparation_row = QHBoxLayout()
        layout.addLayout(preparation_row)
        self.buttons = {}
        for title, name in [('搜索推荐起点', 'analyze'), ('移动到推荐起点', 'move')]:
            button = QPushButton(title)
            button.clicked.connect(lambda checked=False, n=name: self.call(n))
            self.buttons[name] = button
            preparation_row.addWidget(button)
        plot_button = QPushButton('跟踪曲线 / 导出')
        plot_button.clicked.connect(self.show_plot)
        preparation_row.addWidget(plot_button)
        self.preparation = QLabel('起点准备：等待搜索服务')
        self.preparation.setWordWrap(True)
        layout.addWidget(self.preparation)
        row = QHBoxLayout()
        layout.addLayout(row)
        for text, name in [('更新预览', 'preview'), ('开始跟踪', 'start'), ('停止', 'stop')]:
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, n=name: self.call(n))
            if name == 'stop':
                button.setStyleSheet('background:#bd3636; color:white; font-weight:bold; padding:12px')
            self.buttons[name] = button
            row.addWidget(button)
        self.readiness = QLabel('等待跟踪节点就绪…')
        self.readiness.setWordWrap(True)
        layout.addWidget(self.readiness)
        self.state = QLabel('等待节点状态')
        self.result = QLabel('修改参数后请先更新预览；开始执行使用已预览的参数。')
        self.metrics = QLabel('')
        for label in (self.state, self.result, self.metrics):
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addWidget(QLabel('操作：设置参数 → 搜索推荐起点 → 检查紫色曲线 → 移动到推荐起点\n→ 更新预览检查实际起点 → 开始跟踪。也可直接检查当前起点。\n搜索和预览不会移动机械臂；推荐起点是有限候选中的较优解。'))
        layout.addWidget(QLabel('RViz：青色=目标曲线，橙色=实际 TCP，黄色=当前目标，红线=位置误差。\n停止后仍有控制柜前瞻点和减速尾段；此按钮不是硬件急停。'))
        self.events.state.connect(self.state.setText)
        self.events.result.connect(self.result.setText)
        self.events.metrics.connect(self.metrics.setText)
        self.events.ready.connect(self.unlock)
        self.events.readiness.connect(self.on_readiness)
        self.events.previewed.connect(self.on_previewed)
        self.events.analyzed.connect(self.on_analyzed)
        self.events.preparation.connect(self.on_preparation)
        for box in self.fields.values():
            signal = box.currentTextChanged if isinstance(box, QComboBox) else box.valueChanged
            signal.connect(self.parameters_changed)
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self.refresh_buttons)
        self.watchdog.start(200)
        self.refresh_buttons()
        self.events.closed.connect(self.finish_close)
        self.subscribers = [
            rospy.Subscriber(self.ns + '/state', String, lambda m: self.events.state.emit(m.data)),
            rospy.Subscriber(self.ns + '/readiness', String, lambda m: self.events.readiness.emit(m.data)),
            rospy.Subscriber(self.ns + '/metrics', Float64MultiArray, self.on_metrics),
            rospy.Subscriber(self.prep_ns + '/state', String, lambda m: self.events.preparation.emit(m.data)),
            rospy.Subscriber(self.ns + '/samples', Float64MultiArray, lambda m: self.plot.sample_received.emit(list(m.data)), queue_size=200)]

    def unlock(self):
        self.busy = False
        for box in self.fields.values():
            box.setEnabled(True)
        self.refresh_buttons()

    def parameters_changed(self, *_):
        self.params_dirty = True
        self.preparation_dirty = True
        self.refresh_buttons()

    def on_previewed(self):
        self.params_dirty = False
        self.plot.set_plane(self.fields['plane'].currentText())

    def on_analyzed(self):
        self.preparation_dirty = False

    def on_preparation(self, text):
        self.preparation_kind, _, detail = text.partition(': ')
        self.preparation_time = time.monotonic()
        self.preparation.setText('起点准备：' + detail)
        self.refresh_buttons()

    def show_plot(self):
        self.plot.show()
        self.plot.raise_()

    def on_readiness(self, text):
        self.readiness_kind, _, self.readiness_detail = text.partition(': ')
        self.readiness_time = time.monotonic()
        self.refresh_buttons()

    def refresh_buttons(self):
        fresh = time.monotonic() - self.readiness_time < 1.0
        prep_fresh = time.monotonic() - self.preparation_time < 1.0
        prep_active = prep_fresh and self.preparation_kind in ('SEARCHING', 'MOVING', 'STOPPING')
        can_preview = not prep_active and fresh and self.readiness_kind in ('PREVIEW_READY', 'START_READY')
        can_start = not prep_active and fresh and self.readiness_kind == 'START_READY' and not self.params_dirty
        self.buttons['analyze'].setEnabled(can_preview and prep_fresh and not self.busy)
        self.buttons['move'].setEnabled(can_preview and prep_fresh and self.preparation_kind == 'READY' and not self.preparation_dirty and not self.busy)
        for box in self.fields.values():
            box.setEnabled(not self.busy and not prep_active)
        self.buttons['preview'].setEnabled(can_preview and not self.busy)
        self.buttons['start'].setEnabled(can_start and not self.busy)
        if not fresh:
            text = '等待节点就绪：未收到实时状态，请检查跟踪节点是否运行'
        elif self.params_dirty and can_preview:
            text = '参数已修改，请点击“更新预览”'
        else:
            text = self.readiness_detail
        self.readiness.setText('准备状态：' + text)
        self.readiness.setStyleSheet('color: ' + ('#237a35' if can_start else '#9a5700'))
        self.buttons['preview'].setToolTip('记录当前 TCP 起点并生成曲线，不移动机械臂' if can_preview else text)
        self.buttons['start'].setToolTip('执行已预览的轨迹' if can_start else text)

    def on_metrics(self, msg):
        if len(msg.data) == 5:
            t, error, peak, rms, speed = msg.data
            self.events.metrics.emit('时间 %.1f s | 误差 %.2f mm\n峰值 %.2f mm | RMS %.2f mm | 实际速度 %.2f mm/s' %
                                     (t, error*1000, peak*1000, rms*1000, speed*1000))

    def call(self, name, closing=False):
        values = {key: (box.currentText() if isinstance(box, QComboBox) else box.value())
                  for key, box in self.fields.items()} if name in ('preview', 'analyze') else {}
        self.busy = True
        for box in self.fields.values():
            box.setEnabled(False)
        self.refresh_buttons()
        self.result.setText({'preview': '正在检查当前起点和完整曲线…', 'analyze': '正在搜索起点，可能需要数秒…', 'move': '正在复查并请求移动…', 'start': '正在请求开始…', 'stop': '正在请求停止…'}[name])

        def work():
            try:
                for key, value in values.items():
                    rospy.set_param(self.ns + '/' + key, value)
                if name == 'stop':
                    # Always attempt both stops; a preparation error must not skip tracker stop.
                    try:
                        rospy.wait_for_service(self.prep_ns + '/stop', timeout=.5)
                        prep_stop = rospy.ServiceProxy(self.prep_ns + '/stop', Trigger)()
                        if not prep_stop.success:
                            rospy.ServiceProxy('/cartesian_jog/set_enabled', SetBool)(False)
                    except (rospy.ROSException, rospy.ServiceException):
                        try:
                            rospy.ServiceProxy('/cartesian_jog/set_enabled', SetBool)(False)
                        except rospy.ServiceException as exc:
                            self.events.result.emit('起点停止未确认：' + str(exc))
                service_ns = self.prep_ns if name in ('analyze', 'move') else self.ns
                rospy.wait_for_service(service_ns + '/' + name, timeout=1.)
                response = rospy.ServiceProxy(service_ns + '/' + name, Trigger)()
                if name == 'analyze' and response.success:
                    self.events.analyzed.emit()
                if name == 'preview' and response.success:
                    self.events.previewed.emit()
                self.events.result.emit(('成功：' if response.success else '拒绝：') + response.message)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                self.events.result.emit(str(exc))
            finally:
                self.events.ready.emit()
                if closing:
                    self.events.closed.emit()
        threading.Thread(target=work, daemon=True).start()

    def closeEvent(self, event):
        if self.close_ready:
            event.accept()
            return
        event.ignore()
        if not self.close_pending:
            self.close_pending = True
            self.call('stop', closing=True)

    def finish_close(self):
        self.close_ready = True
        self.plot.timer.stop()
        self.plot.hide()
        self.close()


if __name__ == '__main__':
    rospy.init_node('trajectory_tracking_panel', disable_signals=True)
    app = QApplication(sys.argv)
    panel = Panel()
    panel.show()
    sys.exit(app.exec_())
