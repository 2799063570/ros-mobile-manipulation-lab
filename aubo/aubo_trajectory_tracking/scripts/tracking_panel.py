#!/usr/bin/env python3
"""Small Qt client; ROS service calls run off the GUI thread so Stop stays responsive."""
import sys
import threading
import rospy
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtWidgets import (QApplication, QWidget, QVBoxLayout, QFormLayout,
                                        QHBoxLayout, QComboBox, QDoubleSpinBox, QSpinBox,
                                        QPushButton, QLabel)


class Events(QObject):
    state = Signal(str)
    result = Signal(str)
    metrics = Signal(str)
    ready = Signal()
    closed = Signal()


class Panel(QWidget):
    def __init__(self):
        super().__init__()
        self.ns = rospy.get_param('~tracker_ns', '/trajectory_tracking')
        self.events = Events()
        self.close_pending = False
        self.close_ready = False
        self.setWindowTitle('AUBO 轨迹跟踪')
        self.resize(460, 580)
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
            ('max_speed', '速度上限 (m/s)', .001, .10, .015)]:
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
        row = QHBoxLayout()
        layout.addLayout(row)
        self.buttons = []
        for text, name in [('更新预览', 'preview'), ('开始跟踪', 'start'), ('停止', 'stop')]:
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, n=name: self.call(n))
            if name == 'stop':
                button.setStyleSheet('background:#bd3636; color:white; font-weight:bold; padding:12px')
            else:
                self.buttons.append(button)
            row.addWidget(button)
        self.state = QLabel('等待节点状态')
        self.result = QLabel('修改参数后请先更新预览；开始执行使用已预览的参数。')
        self.metrics = QLabel('')
        for label in (self.state, self.result, self.metrics):
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addWidget(QLabel('RViz：青色=目标曲线，橙色=实际 TCP，黄色=当前目标，红线=位置误差。\n停止后仍有控制柜前瞻点和减速尾段；此按钮不是硬件急停。'))
        self.events.state.connect(self.state.setText)
        self.events.result.connect(self.result.setText)
        self.events.metrics.connect(self.metrics.setText)
        self.events.ready.connect(self.unlock)
        self.events.closed.connect(self.finish_close)
        self.subscribers = [
            rospy.Subscriber(self.ns + '/state', String, lambda m: self.events.state.emit(m.data)),
            rospy.Subscriber(self.ns + '/metrics', Float64MultiArray, self.on_metrics)]

    def unlock(self):
        for button in self.buttons:
            button.setEnabled(True)

    def on_metrics(self, msg):
        if len(msg.data) == 5:
            t, error, peak, rms, speed = msg.data
            self.events.metrics.emit('时间 %.1f s | 误差 %.2f mm\n峰值 %.2f mm | RMS %.2f mm | 实际速度 %.2f mm/s' %
                                     (t, error*1000, peak*1000, rms*1000, speed*1000))

    def call(self, name, closing=False):
        values = {key: (box.currentText() if isinstance(box, QComboBox) else box.value())
                  for key, box in self.fields.items()} if name == 'preview' else {}
        for button in self.buttons:
            button.setEnabled(False)

        def work():
            try:
                for key, value in values.items():
                    rospy.set_param(self.ns + '/' + key, value)
                rospy.wait_for_service(self.ns + '/' + name, timeout=1.)
                response = rospy.ServiceProxy(self.ns + '/' + name, Trigger)()
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
        self.close()


if __name__ == '__main__':
    rospy.init_node('trajectory_tracking_panel', disable_signals=True)
    app = QApplication(sys.argv)
    panel = Panel()
    panel.show()
    sys.exit(app.exec_())
