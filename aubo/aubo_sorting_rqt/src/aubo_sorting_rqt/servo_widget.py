"""Servo tuning workbench. Editing or previewing never sends ROS commands."""
from collections import deque
import datetime
import math
import time

from python_qt_binding.QtCore import Qt, Signal, QPointF, QRectF
from python_qt_binding.QtGui import QColor, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QScrollArea,
    QSplitter, QTabWidget, QVBoxLayout, QWidget)

from .servo_model import (PARAMETERS, POSE_KEYS, STATES, action_gates, downward_draft,
                           quaternion_rpy, segment_distance, tool_axis, visibility_draft)
from .widget import CameraView


class ErrorPlot(QWidget):
    def __init__(self):
        super(ErrorPlot, self).__init__()
        self.samples = deque(maxlen=600)
        self.setMinimumSize(320, 230)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#f5f9fc'))
        box = QRectF(48, 32, max(1, self.width()-64), max(1, self.height()-66))
        recent = [s for s in self.samples if s[0] >= time.monotonic()-60.]
        scale = max([10.]+[abs(v)*1000 for _, value in recent if value for v in value])
        p.setPen(QPen(QColor('#dbe5ed'), 1))
        for i in range(5):
            y = box.top()+box.height()*i/4
            p.drawLine(QPointF(box.left(), y), QPointF(box.right(), y))
        p.setPen(QColor('#526d82'))
        p.drawText(6, 20, '误差 / mm    最近 60 s（数据无效时断线）')
        p.drawText(2, int(box.top()+6), '{:.0f}'.format(scale))
        p.drawText(2, int(box.center().y()+5), '0')
        p.drawText(2, int(box.bottom()), '-{:.0f}'.format(scale))
        colors = ('#d35454', '#179b80', '#3986cb')
        for axis, color in enumerate(colors):
            p.setPen(QPen(QColor(color), 2))
            p.drawText(55+axis*70, self.height()-9, 'XYZ'[axis])
            line = QPolygonF()
            for stamp, values in recent:
                if values is None:
                    if len(line) > 1: p.drawPolyline(line)
                    line = QPolygonF()
                    continue
                x = box.right()-(time.monotonic()-stamp)/60.*box.width()
                y = box.center().y()-values[axis]*1000/scale*box.height()/2
                line.append(QPointF(x, y))
            if len(line) > 1: p.drawPolyline(line)


class SightView(QWidget):
    def __init__(self):
        super(SightView, self).__init__()
        self.scene = {}
        self.setMinimumSize(360, 280)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#f5f9fc'))
        scene = self.scene
        if not scene.get('camera'):
            p.setPen(QColor('#526d82'))
            p.drawText(self.rect(), Qt.AlignCenter, '等待相机、机械臂和目标 TF')
            return
        camera, target = scene['camera'], scene['target']
        dx, dy = camera[0]-target[0], camera[1]-target[1]
        length = max(.001, math.hypot(dx, dy))
        project = lambda v: ((v[0]-target[0])*dx/length+(v[1]-target[1])*dy/length, v[2])
        points = scene.get('links', [])+[camera, target]+scene.get('tool', [])
        xy = [project(v) for v in points]
        xmin, xmax = min(v[0] for v in xy)-.12, max(v[0] for v in xy)+.12
        zmin, zmax = min(v[1] for v in xy)-.08, max(v[1] for v in xy)+.12
        box = QRectF(28, 28, max(1, self.width()-56), max(1, self.height()-70))
        def screen(v):
            x, z = project(v)
            return QPointF(box.left()+(x-xmin)/(xmax-xmin)*box.width(),
                           box.bottom()-(z-zmin)/(zmax-zmin)*box.height())
        links = scene.get('links', [])
        p.setPen(QPen(QColor('#829bad'), 9, Qt.SolidLine, Qt.RoundCap))
        if len(links) > 1: p.drawPolyline(QPolygonF([screen(v) for v in links]))
        p.setPen(QPen(QColor('#dc6b45'), 2, Qt.DashLine))
        p.drawLine(screen(camera), screen(target))
        tool = scene.get('tool', [])
        if len(tool) == 2:
            p.setPen(QPen(QColor('#824eba'), 7, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(screen(tool[0]), screen(tool[1]))
        for point, label, color in ((camera, '相机', '#267fa4'), (target, '物体表面', '#d77823')):
            p.setPen(QColor(color)); p.setBrush(QColor(color))
            pos = screen(point)
            p.drawEllipse(pos, 5, 5)
            p.drawText(pos+QPointF(8, -7), label)
        if tool:
            p.setPen(QColor('#824eba'))
            p.drawText(screen(tool[0])+QPointF(8, 14), '草稿 TCP / 夹爪')
        p.setPen(QColor('#526d82'))
        p.drawText(12, self.height()-22, '侧视投影：橙虚线=视线，灰=当前连杆，紫=草稿夹爪')
        p.drawText(12, self.height()-5, '距离判断使用三维包络；未验证 IK、全臂候选姿态或避障路径')


class ServoWidget(QWidget):
    command_requested = Signal(str)
    parameters_requested = Signal(object)
    class_requested = Signal(str)
    refresh_requested = Signal()
    export_requested = Signal()

    def __init__(self):
        super(ServoWidget, self).__init__()
        self.setWindowTitle('AUBO 视觉伺服调试台')
        self.setObjectName('AuboServoPanel')
        self.resize(1200, 860)
        self._config, self._data, self._dirty = {}, {}, set()
        self._filling, self._last_sample = False, None
        self.setStyleSheet('''
            QWidget#AuboServoPanel { background: #edf3f7; color: #243c50; }
            QGroupBox { background: white; border: 1px solid #d5e1e9; border-radius: 6px;
                        margin-top: 12px; padding: 12px 8px 8px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { min-height: 30px; padding: 3px 12px; border: 1px solid #adc4d3;
                          border-radius: 4px; background: white; }
            QPushButton#start { background: #087e8b; color: white; }
            QPushButton#stop { background: #ba3b3b; color: white; }
            QPushButton:disabled { color: #8f9fae; background: #e2e9ed; }
            QDoubleSpinBox { min-height: 25px; padding: 2px 4px; }
            QPlainTextEdit { background: #f7fafc; border: 1px solid #d5e1e9; }
        ''')
        root = QVBoxLayout(self)
        title = QLabel('AUBO  /  视觉伺服调试台')
        title.setStyleSheet('font-size: 24px; font-weight: 700; padding: 6px 0;')
        root.addWidget(title)
        self.status_label = QLabel('等待视觉伺服连接')
        self.status_label.setStyleSheet('font-size: 16px; font-weight: 600;')
        root.addWidget(self.status_label)
        self.health_label = QLabel('反馈 —    目标 —    图像 —')
        root.addWidget(self.health_label)
        self.observation_label = QLabel('观测位：未启用固定观测姿态')
        self.observation_label.setWordWrap(True)
        root.addWidget(self.observation_label)
        controls = QHBoxLayout()
        self.buttons = {}
        for key, label in (('start', '使能视觉伺服'), ('stop', '停止伺服'), ('reset', '复位保护'),
                           ('vision_on', '启用识别'), ('vision_off', '暂停识别')):
            button = QPushButton(label)
            button.setObjectName(key)
            button.clicked.connect(lambda checked=False, key=key: self.command_requested.emit(key))
            self.buttons[key] = button
            controls.addWidget(button)
        self.target_combo = QComboBox()
        self.target_combo.setEditable(True)
        self.target_combo.addItems(['can', 'bottle', 'box'])
        controls.addWidget(self.target_combo)
        self.class_button = QPushButton('切换目标类别')
        self.class_button.clicked.connect(lambda: self.class_requested.emit(self.target_combo.currentText().strip()))
        controls.addWidget(self.class_button)
        root.addLayout(controls)
        self.notice = QLabel('打开界面仅订阅；使能、参数应用和类别切换均需显式点击。')
        self.notice.setWordWrap(True); self.notice.setTextFormat(Qt.PlainText)
        root.addWidget(self.notice)
        tabs = QTabWidget()
        root.addWidget(tabs)

        tracking = QWidget(); layout = QVBoxLayout(tracking)
        splitter = QSplitter(Qt.Horizontal)
        self.camera = CameraView(); self.plot = ErrorPlot()
        splitter.addWidget(self.camera); splitter.addWidget(self.plot)
        layout.addWidget(splitter, 1)
        self.position_label = QLabel('实际 TCP / 期望 TCP / XYZ 误差：—')
        self.position_label.setWordWrap(True); self.position_label.setTextFormat(Qt.PlainText)
        layout.addWidget(self.position_label)
        self.perception_label = QLabel('目标类别：等待感知状态')
        layout.addWidget(self.perception_label)
        tabs.addTab(tracking, '跟踪画面与误差')

        tuning = QWidget(); tuning_layout = QVBoxLayout(tuning)
        tuning_layout.addWidget(QLabel('数值以 mm、mm/s、角度显示；右侧为服务器实际值。编辑不会自动生效。'))
        columns = QHBoxLayout(); tuning_layout.addLayout(columns)
        self.editors, self.actual = {}, {}
        self._groups = []
        for group_name, pose in (('闭环与速度', False), ('目标偏移与 TCP 姿态', True)):
            group = QGroupBox(group_name); grid = QGridLayout(group)
            self._groups.append(grid)
            columns.addWidget(group)
            row = 0
            for key, label, low, high, step, decimals, scale in PARAMETERS:
                if (key in POSE_KEYS) != pose: continue
                editor = QDoubleSpinBox()
                editor.setRange(low, high); editor.setSingleStep(step); editor.setDecimals(decimals)
                editor.valueChanged.connect(lambda value, key=key: self._edit(key))
                self.editors[key] = editor
                actual = QLabel('—'); self.actual[key] = actual
                grid.addWidget(QLabel(label), row, 0); grid.addWidget(editor, row, 1); grid.addWidget(actual, row, 2)
                row += 1
            if pose:
                self.orientation = QCheckBox('启用姿态闭环')
                self.orientation.toggled.connect(lambda value: self._edit('use_orientation_control'))
                grid.addWidget(self.orientation, row, 0, 1, 2)
                self.actual['use_orientation_control'] = QLabel('—')
                grid.addWidget(self.actual['use_orientation_control'], row, 2)
            button = QPushButton('应用姿态与偏移（先禁用）' if pose else '应用控制参数')
            button.clicked.connect(lambda checked=False, pose=pose: self._apply(pose))
            grid.addWidget(button, row+1, 0, 1, 3)
            if pose: self.pose_apply = button
            else: self.control_apply = button
            grid.setRowStretch(row+2, 1)
        actions = QHBoxLayout()
        refresh = QPushButton('重读实际参数（覆盖草稿）')
        refresh.clicked.connect(lambda checked=False: self.refresh_requested.emit())
        export = QPushButton('导出服务器实际参数 YAML')
        export.clicked.connect(lambda checked=False: self.export_requested.emit())
        actions.addWidget(refresh); actions.addWidget(export)
        tuning_layout.addLayout(actions)
        self.parameter_note = QLabel('姿态和目标偏移只能在禁用、停稳后应用；界面关闭不会停止伺服。')
        self.parameter_note.setWordWrap(True); tuning_layout.addWidget(self.parameter_note)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(tuning)
        tabs.addTab(scroll, '在线参数与姿态')

        visibility = QWidget(); view_layout = QVBoxLayout(visibility)
        self.sight = SightView(); view_layout.addWidget(self.sight)
        self.visibility_label = QLabel('视线余量：等待几何数据')
        self.visibility_label.setWordWrap(True); view_layout.addWidget(self.visibility_label)
        self.pose_label = QLabel('当前 TCP 姿态：—'); view_layout.addWidget(self.pose_label)
        drafts = QHBoxLayout()
        self.height = QDoubleSpinBox(); self.height.setRange(50, 500); self.height.setValue(150); self.height.setSuffix(' mm 悬停')
        self.lateral = QDoubleSpinBox(); self.lateral.setRange(0, 300); self.lateral.setValue(80); self.lateral.setSuffix(' mm 侧移')
        drafts.addWidget(self.height); drafts.addWidget(self.lateral)
        downward = QPushButton('朝下姿态草稿（Yaw 90°）')
        downward.clicked.connect(lambda: self.draft(downward_draft()))
        self.sight_draft = QPushButton('按当前视线生成避遮挡草稿')
        self.sight_draft.clicked.connect(self._visibility_draft)
        drafts.addWidget(downward); drafts.addWidget(self.sight_draft)
        view_layout.addLayout(drafts)
        note = QLabel('建议：先悬停 150 mm，向远离相机的一侧偏移约 80 mm；TCP Z 朝下，Yaw 随视线选取。\n'
                      '夹爪模型 X≈24 mm、Y≈86 mm；让长 Y 轴朝向相机水平视线可减小侧向轮廓。\n'
                      '草稿只修改编辑框，不移动机械臂。侧移适合跟踪悬停，正式抓取前需另行对齐。')
        note.setWordWrap(True); view_layout.addWidget(note)
        tabs.addTab(visibility, '相机视线与姿态助手')
        self.log_view = QPlainTextEdit(); self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(400)
        tabs.addTab(self.log_view, '过程记录')

    def log(self, text):
        self.log_view.appendPlainText(datetime.datetime.now().strftime('%H:%M:%S')+'  '+str(text))
        self.notice.setText(str(text))

    def _edit(self, key):
        if not self._filling: self._dirty.add(key)
        self._preview()

    def values(self):
        result = {key: self.editors[key].value()/scale for key, _, _, _, _, _, scale in PARAMETERS}
        result['use_orientation_control'] = self.orientation.isChecked()
        return result

    def _apply(self, pose):
        values = self.values()
        self.parameters_requested.emit({key: values[key] for key in self._dirty if (key in POSE_KEYS) == pose})

    def configuration(self, config, force=False):
        self._config = dict(config)
        self._filling = True
        for key, _, _, _, _, decimals, scale in PARAMETERS:
            if key not in config: continue
            self.actual[key].setText(('{:.'+str(decimals)+'f}').format(config[key]*scale))
            if force or key not in self._dirty:
                editor = self.editors[key]
                value = config[key]*scale
                editor.setRange(min(editor.minimum(), value), max(editor.maximum(), value))
                editor.setValue(value)
        if 'use_orientation_control' in config:
            self.actual['use_orientation_control'].setText('开启' if config['use_orientation_control'] else '关闭')
            if force or 'use_orientation_control' not in self._dirty:
                self.orientation.setChecked(bool(config['use_orientation_control']))
        if force: self._dirty.clear()
        self._filling = False
        self._preview()

    def draft(self, values):
        self._filling = True
        scales = {p[0]: p[-1] for p in PARAMETERS}
        for key, value in values.items():
            if key == 'use_orientation_control': self.orientation.setChecked(value)
            elif key in self.editors: self.editors[key].setValue(value*scales[key])
            self._dirty.add(key)
        self._filling = False
        self.parameter_note.setText('姿态草稿已生成，尚未发送。先禁用并停稳，再到参数页应用姿态与偏移。')
        self.log(self.parameter_note.text())
        self._preview()

    def _visibility_draft(self):
        view = self._data.get('visibility', {})
        try:
            if time.monotonic()-view.get('at', 0.) > .75:
                raise ValueError('视线数据过期，等待新目标和完整 TF')
            yaw = quaternion_rpy(view['rotation'])[2]
            self.draft(visibility_draft(view['camera'], view['target'], self.height.value()/1000,
                                        self.lateral.value()/1000, yaw))
        except (ValueError, KeyError) as error:
            self.log(str(error))

    def _preview(self):
        if not hasattr(self, 'sight'): return
        view = self._data.get('visibility', {})
        if not view.get('camera') or time.monotonic()-view.get('at', 0.) > .75:
            self.sight.scene = {}; self.sight.update()
            self.visibility_label.setText('视线余量未知：'+view.get('detail', '等待目标和完整 TF'))
            self.pose_label.setText('当前 TCP 姿态：—')
            return
        try:
            values = self.values()
            rpy = quaternion_rpy(view['rotation'])
            self.pose_label.setText('当前 TCP RPY：{:.1f}° / {:.1f}° / {:.1f}°'.format(*(v*180/math.pi for v in rpy)))
            axis = tool_axis(tuple(values['desired_'+key] for key in ('roll', 'pitch', 'yaw'))
                             if values['use_orientation_control'] else rpy)
            goal = tuple(view['target'][i]+values['target_offset_'+'xyz'[i]] for i in range(3))
            body = tuple(goal[i]-.12*axis[i] for i in range(3))
            margin = segment_distance(view['camera'], view['target'], goal, body)-.06
            current = view['clearance']
            self.visibility_label.setText('当前全臂近似视线余量：{:+.1f} mm   |   草稿夹爪余量：{:+.1f} mm\n'
                '负值=包络与视线交叠；正值仅表示近似未交叠。草稿未验证全臂 IK / 碰撞。'.format(current*1000, margin*1000))
            self.visibility_label.setStyleSheet('color: '+('#b33c3c' if min(current, margin) <= 0 else '#487387'))
            self.sight.scene = dict(view, tool=[goal, body]); self.sight.update()
        except (ValueError, KeyError):
            self.sight.scene = {}; self.sight.update()

    def render(self, data):
        self._data = data
        now = time.monotonic(); status = data.get('status') or {}
        fresh = data.get('connected') and now-data.get('status_at', 0.) < .75
        state = data.get('state', '')
        cell = data.get('cell', {})
        observation = cell.get('initial_search_posture', [])
        if cell.get('initial_search_enabled') and len(observation) == 6:
            self.observation_label.setText('固定观测关节角 / °：'+ ' / '.join('{:.2f}'.format(v*180/math.pi) for v in observation)+
                                           '；使能先回此位，目标丢失衰减后返回。')
        else:
            self.observation_label.setText('观测位：未启用固定观测姿态')
        self.status_label.setText('视觉伺服：'+(STATES.get(state, state) if fresh else '连接或遥测过期'))
        gates = action_gates(data, now)
        for key in ('start', 'stop', 'reset'): self.buttons[key].setEnabled(gates[key])
        for key in ('vision_on', 'vision_off'): self.buttons[key].setEnabled(bool(data.get('perception_controls')))
        self.class_button.setEnabled(gates['pose'] and bool(data.get('perception_controls')))
        self.control_apply.setEnabled(gates['tune']); self.pose_apply.setEnabled(gates['pose'])
        view = data.get('visibility', {})
        self.sight_draft.setEnabled(now-view.get('at', 0.) < .75)
        age = lambda key: '{:.0f} ms'.format(max(0., status.get(key, 0.))*1000) if fresh else '—'
        self.health_label.setText('反馈：{} {}    目标：{} {}    图像接收：{}'.format(
            '有效' if fresh and status.get('feedback_valid') else '无效', age('feedback_age'),
            '有效' if fresh and status.get('target_valid') else '无效', age('target_age'),
            '正常' if now-data.get('image_at', 0.) < 1. else '等待 / 过期'))
        self.camera.image = data.get('image') if now-data.get('image_at', 0.) < 1. else None
        self.camera.update()
        valid = fresh and status.get('error_valid')
        if valid:
            mm = lambda key: ' / '.join('{:.1f}'.format(v*1000) for v in status[key])
            self.position_label.setText('实际 TCP XYZ / mm：{}\n期望 TCP XYZ / mm：{}\n'
                                        '误差 XYZ / mm：{}'.format(mm('tcp'), mm('goal'), mm('error')))
        else:
            self.position_label.setText('实际 TCP / 期望 TCP / XYZ 误差：—（遥测或几何无效）')
        sample = data.get('status_at') if valid else None
        if sample != self._last_sample or not valid:
            self.plot.samples.append((now, tuple(status['error']) if valid else None))
            self._last_sample = sample
        self.plot.update()
        self.perception_label.setText('感知：'+data.get('perception', '等待状态')+
                                      '    当前类别：'+data.get('selected_class', '—')+
                                      ('    置信度：{:.2f}'.format(data['confidence']) if fresh and status.get('target_valid') and
                                       math.isfinite(data.get('confidence', float('nan'))) else '    置信度：—')+
                                      ('    等待切换后的新目标，暂不允许使能' if data.get('selection_pending') else ''))
        self._preview()
