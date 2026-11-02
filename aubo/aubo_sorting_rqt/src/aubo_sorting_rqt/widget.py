"""Qt presentation, independent of ROS IO so it can be exercised offline."""
import datetime
import math
import time

from python_qt_binding.QtCore import Qt, Signal, QRectF
from python_qt_binding.QtGui import QColor, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QDoubleSpinBox, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit, QPushButton,
    QSpinBox, QSplitter, QTableWidget, QTableWidgetItem, QTabWidget,
    QVBoxLayout, QWidget,
)

from .model import ACTIONS, IDLE_STATES, PARAMETERS, PHASES, QUEUE_STATES, STATES, available_actions, process_step, state_code

COLORS = {'bottle': '#059669', 'can': '#0284c7', 'box': '#b45309'}


class CameraView(QWidget):
    def __init__(self):
        super(CameraView, self).__init__()
        self.image = None
        self.setMinimumSize(260, 190)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#172b3a'))
        if self.image is None:
            painter.setPen(QColor('#b6c8d4'))
            painter.drawText(self.rect(), Qt.AlignCenter, '等待相机画面')
        else:
            size = self.image.size().scaled(self.size(), Qt.KeepAspectRatio)
            target = QRectF((self.width()-size.width())/2, (self.height()-size.height())/2,
                            size.width(), size.height())
            painter.drawImage(target, self.image)


class QueueMap(QWidget):
    def __init__(self):
        super(QueueMap, self).__init__()
        self.rows = []
        self.cell = {}
        self.setMinimumSize(260, 190)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#f7fafc'))
        bounds = QRectF(42, 16, max(1, self.width()-65), max(1, self.height()-54))
        center, size = self.cell.get('table_center'), self.cell.get('table_size')
        if center and size:
            xmin, xmax = center[0]-size[0]/2, center[0]+size[0]/2
            ymin, ymax = center[1]-size[1]/2, center[1]+size[1]/2
        else:
            xmin, xmax, ymin, ymax = -.5, .4, -1.2, -.1
        painter.setPen(QPen(QColor('#d9e3ea'), 1))
        for i in range(6):
            x = bounds.left()+bounds.width()*i/5
            y = bounds.top()+bounds.height()*i/5
            painter.drawLine(int(x), int(bounds.top()), int(x), int(bounds.bottom()))
            painter.drawLine(int(bounds.left()), int(y), int(bounds.right()), int(y))
        painter.setPen(QColor('#617586'))
        painter.drawText(5, 16, 'Y / m')
        painter.drawText(int(bounds.left()), self.height()-8, '{:.2f}'.format(xmin))
        painter.drawText(self.width()-68, self.height()-8, '{:.2f} X'.format(xmax))
        painter.drawText(2, int(bounds.top()+16), '{:.2f}'.format(ymax))
        painter.drawText(2, int(bounds.bottom()), '{:.2f}'.format(ymin))
        if xmax <= xmin or ymax <= ymin:
            return
        for row in self.rows:
            x, y, _ = row['position']
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                continue
            px = bounds.left()+(x-xmin)/(xmax-xmin)*bounds.width()
            py = bounds.bottom()-(y-ymin)/(ymax-ymin)*bounds.height()
            color = COLORS.get(row['category'], '#64748b')
            if row.get('status') in ('done', 'review'):
                color = '#aab5bf'
            painter.setBrush(QColor(color))
            painter.setPen(QPen(QColor('#152c3e'), 3 if row.get('status') == 'reserved' else 1))
            painter.drawEllipse(QRectF(px-6, py-6, 12, 12))
            painter.setPen(QColor('#243b4d'))
            painter.drawText(int(px+9), int(py+4), '#{} {}'.format(row['id'], row['category']))


class SortingWidget(QWidget):
    command_requested = Signal(str)
    parameters_requested = Signal(object)
    refresh_requested = Signal()
    export_requested = Signal()

    def __init__(self):
        super(SortingWidget, self).__init__()
        self.setWindowTitle('AUBO 分拣工作台')
        self.setObjectName('AuboSortingPanel')
        self.resize(1160, 820)
        self._dirty = False
        self._filling = False
        self._rows = None
        self._config = {}
        self.setStyleSheet('''
            QWidget#AuboSortingPanel { background: #edf3f7; color: #233a4c; }
            QGroupBox { background: white; border: 1px solid #d8e2ea; border-radius: 7px;
                        margin-top: 14px; padding: 12px 8px 8px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QPushButton { min-height: 30px; padding: 3px 14px; border-radius: 5px;
                          border: 1px solid #b8cbd9; background: #fff; color: #24465e; }
            QPushButton:hover { background: #e0f2fe; }
            QPushButton:disabled { color: #91a1af; background: #eef2f5; border-color: #dae2e8; }
            QPushButton#start { background: #087e8b; color: white; border-color: #087e8b; }
            QPushButton#stop { background: #b93838; color: white; border-color: #b93838; }
            QPushButton#start:disabled, QPushButton#stop:disabled { background: #ccd7dd; color: #8b9ba7; border-color: #ccd7dd; }
            QTableWidget { border: 0; gridline-color: #e5edf2; background: white; }
            QHeaderView::section { background: #eef4f8; border: 0; padding: 6px; color: #405d72; }
            QPlainTextEdit { background: #f8fafc; border: 1px solid #d8e2ea; border-radius: 4px; }
            QDoubleSpinBox, QSpinBox { min-height: 26px; padding: 2px 5px; }
        ''')
        root = QVBoxLayout(self)
        title = QLabel('AUBO  /  分拣工作台')
        title.setStyleSheet('font-size: 23px; font-weight: 700; padding: 6px 0;')
        root.addWidget(title)
        self.status = QLabel('等待分拣节点连接')
        self.status.setStyleSheet('font-size: 16px; font-weight: 600;')
        root.addWidget(self.status)
        self.detail = QLabel('先连接任务，再前往观察位并启动队列分拣。')
        self.detail.setWordWrap(True)
        root.addWidget(self.detail)
        self.failure = QLabel()
        self.failure.setTextFormat(Qt.PlainText)
        self.failure.setWordWrap(True)
        self.failure.setStyleSheet('color: #a32929; background: #fff0f0; padding: 7px;')
        root.addWidget(self.failure)
        steps = QHBoxLayout()
        self.steps = []
        for index, text in enumerate(('观察', '识别入队', '抓取', '放置', '返位')):
            label = QLabel('{}  {}'.format(index+1, text))
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(34)
            self.steps.append(label)
            steps.addWidget(label)
        root.addLayout(steps)
        self.action_detail = QLabel('当前动作：等待进度')
        root.addWidget(self.action_detail)
        self.notice = QLabel('界面只读取状态；操作按钮会向任务发送指令。')
        self.notice.setTextFormat(Qt.PlainText)
        self.notice.setWordWrap(True)
        self.notice.setStyleSheet('color: #346480;')
        root.addWidget(self.notice)
        self.health = QLabel('相机：等待  ·  机械臂反馈：等待')
        root.addWidget(self.health)
        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)
        self._overview()
        self._parameters()
        self._history()
        controls = QHBoxLayout()
        self.buttons = {}
        for key, text in ACTIONS:
            button = QPushButton(text)
            button.setObjectName(key)
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, key=key: self.command_requested.emit(key))
            self.buttons[key] = button
            controls.addWidget(button)
        root.addLayout(controls)
        hint = QLabel('操作顺序：前往观察位 → 等待就绪 → 开始队列分拣。停止任务为软件停止，现场急停请使用硬件按钮。')
        hint.setWordWrap(True)
        hint.setStyleSheet('color: #60798c; font-size: 11px;')
        root.addWidget(hint)

    def _overview(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        splitter = QSplitter(Qt.Horizontal)
        camera_group = QGroupBox('识别画面')
        camera_layout = QVBoxLayout(camera_group)
        self.camera = CameraView()
        camera_layout.addWidget(self.camera)
        self.map_group = QGroupBox('工作台目标 · 平面投影')
        map_layout = QVBoxLayout(self.map_group)
        self.map = QueueMap()
        map_layout.addWidget(self.map)
        splitter.addWidget(camera_group)
        splitter.addWidget(self.map_group)
        layout.addWidget(splitter, 1)
        self.queue_summary = QLabel('实例队列为空')
        layout.addWidget(self.queue_summary)
        self.mode = QLabel('等待读取任务模式')
        self.mode.setWordWrap(True)
        layout.addWidget(self.mode)
        self.table = QTableWidget(0, 11)
        self.table.setHorizontalHeaderLabels(('入队 ID', '类别', '状态', 'X / m', 'Y / m', 'Z / m',
                                             '旋转角度 / °', '夹取宽度 / mm', '检测置信度', '确认帧数', '观测年龄 / s'))
        self.table.horizontalHeaderItem(6).setToolTip('缓存物体短边在队列坐标系 XY 平面内相对 X 轴的角度；TCP 还会叠加配置的抓取偏航角。')
        self.table.horizontalHeaderItem(7).setToolTip('缓存检测的夹取方向宽度，单位 mm；它是物体几何尺寸。')
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setMinimumSectionSize(65)
        for column in (0, 6, 7, 8, 9, 10):
            self.table.horizontalHeader().setSectionResizeMode(column, QHeaderView.ResizeToContents)
        self.table.verticalHeader().hide()
        self.table.setMinimumHeight(155)
        layout.addWidget(self.table, 1)
        self.tabs.addTab(page, '工位与队列')

    def _parameters(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel('在空闲、就绪或停止后应用。执行中的任务保持当前参数；以下“实际值”以服务器返回结果为准。')
        note.setWordWrap(True)
        layout.addWidget(note)
        grid = QGridLayout()
        grid.addWidget(QLabel('参数'), 0, 0)
        grid.addWidget(QLabel('编辑值'), 0, 1)
        grid.addWidget(QLabel('服务器实际值'), 0, 2)
        self.editors, self.actual = {}, {}
        for index, (key, label, low, high, step, decimals) in enumerate(PARAMETERS, 1):
            editor = QSpinBox() if decimals == 0 else QDoubleSpinBox()
            editor.setRange(low, high)
            editor.setSingleStep(step)
            if decimals:
                editor.setDecimals(decimals)
            editor.setEnabled(False)
            editor.valueChanged.connect(self._mark_dirty)
            editor.setToolTip(key)
            self.editors[key] = editor
            self.actual[key] = QLabel('尚未连接')
            grid.addWidget(QLabel(label), index, 0)
            grid.addWidget(editor, index, 1)
            grid.addWidget(self.actual[key], index, 2)
        layout.addLayout(grid)
        row = QHBoxLayout()
        self.apply_button = QPushButton('应用在线参数')
        self.apply_button.setEnabled(False)
        self.apply_button.clicked.connect(lambda: self.parameters_requested.emit(
            {key: editor.value() for key, editor in self.editors.items() if key in self._config}))
        refresh = QPushButton('读取实际值 / 放弃编辑')
        refresh.clicked.connect(self.refresh_requested.emit)
        export = QPushButton('导出参数记录 YAML')
        export.clicked.connect(self.export_requested.emit)
        row.addWidget(self.apply_button)
        row.addWidget(refresh)
        row.addWidget(export)
        row.addStretch()
        layout.addLayout(row)
        self.parameter_note = QLabel('等待在线参数服务器。')
        self.parameter_note.setWordWrap(True)
        layout.addWidget(self.parameter_note)
        layout.addStretch()
        self.tabs.addTab(page, '在线调参')

    def _history(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.history = QPlainTextEdit()
        self.history.setReadOnly(True)
        self.history.setMaximumBlockCount(400)
        layout.addWidget(self.history)
        self.tabs.addTab(page, '过程记录')

    def _mark_dirty(self, *args):
        if not self._filling:
            self._dirty = True
            self.parameter_note.setText('编辑值尚未应用。')

    def log(self, text):
        self.history.appendPlainText('{}  {}'.format(datetime.datetime.now().strftime('%H:%M:%S'), text))

    def configuration(self, config, force=False):
        first = not self._config
        self._config = dict(config)
        self._filling = True
        for key, editor in self.editors.items():
            if key in config:
                self.actual[key].setText(str(config[key]))
                if force or not self._dirty:
                    editor.setValue(config[key])
        self._filling = False
        if force:
            self._dirty = False
        if first or force:
            missing = [key for key in self.editors if key not in config]
            self.parameter_note.setText('部分队列参数需要重启新版分拣节点后提供。' if missing else '已读取服务器实际值；修改后点击应用。')

    def render(self, data):
        state = state_code(data.get('state', ''))
        connected = data.get('connected', False)
        self.status.setText(STATES.get(state, state or '等待连接') if connected else '分拣节点未连接')
        self.detail.setText(data.get('state', '') if connected else '检查分拣 launch 是否已启动；界面会自动重连。')
        self.detail.setTextFormat(Qt.PlainText)
        failure = data.get('failure', '')
        self.failure.setText(failure)
        self.failure.setVisible(bool(failure))
        active_step = process_step(state, data.get('progress')) if connected else -1
        progress = data.get('progress') or {}
        self.action_detail.setText('当前动作：{}  {}'.format(
            PHASES.get(progress.get('phase'), '等待进度'), progress.get('description', '')))
        self.action_detail.setTextFormat(Qt.PlainText)
        for index, label in enumerate(self.steps):
            color = '#087e8b' if index == active_step else '#dfe9f0'
            label.setStyleSheet('border-radius: 5px; padding: 5px; background: {}; color: {};'.format(
                color, 'white' if index == active_step else '#547086'))
        hardware = data.get('hardware', {})
        hardware_fresh = time.monotonic()-data.get('hardware_at', 0.) < 3.
        fault = hardware_fresh and (hardware.get('feedback_timeout_latched') == 'true' or
                                    hardware.get('stream_fault_latched') == 'true' or
                                    hardware.get('controller_connected') == 'false')
        source = {'push': '推送', 'query_latched': '独立查询', 'timer_query': '定时查询'}.get(
            hardware.get('feedback_source'), '等待')
        camera_ok = time.monotonic()-data.get('image_at', 0.) < 3.
        age = hardware.get('feedback_sample_age_ms', '—') if hardware_fresh else '—'
        self.health.setText('相机：{}  ·  关节反馈：{} / {} ms  ·  控制：{}  ·  {}'.format(
            '画面正常' if camera_ok else '等待图像', source if hardware_fresh else '未接入', age,
            '故障锁停' if fault else ('已连接' if hardware_fresh else '等待诊断'), data.get('summary', '')))
        services = data.get('services', set())
        enabled = available_actions(state, connected, data.get('pending', False), fault)
        queue_mode = (data.get('cell') or {}).get('continuous_sorting')
        if queue_mode is False:
            enabled['start'] = False
        vision=data.get('vision_phase','')
        vision_label='抓放阶段屏蔽' if vision=='masked' else ('预抓取遮挡保护' if
            vision=='approach' and data.get('target_occluded') else ('观测目标' if vision in ('approach','collect') else '未采集'))
        self.mode.setText(('队列模式 · 感知：{} · 累计合并 {} 次重复记录'.format(
            vision_label,data.get('merged_detections_total',0))) if queue_mode else
            '当前任务未启用队列模式；请重启分拣 launch 加载新配置后开始。')
        self.mode.setStyleSheet('color: {};'.format('#52788a' if queue_mode else '#a04b19'))
        for key, button in self.buttons.items():
            button.setEnabled(enabled[key] and key in services)
        config = data.get('config', {})
        if config:
            self.configuration(config)
        else:
            self._config = {}
            for label in self.actual.values():
                label.setText('尚未连接')
        idle = connected and state in IDLE_STATES and not data.get('pending') and not fault
        for key, editor in self.editors.items():
            editor.setEnabled(idle and key in config)
        self.apply_button.setEnabled(idle and bool(config))
        self.camera.image = data.get('image')
        self.camera.update()
        rows = data.get('rows', [])
        self.map.rows = rows
        self.map.cell = data.get('cell', {})
        self.map.update()
        self.map_group.setTitle('工作台目标 · {}'.format(data.get('frame') or '等待坐标系'))
        counts = {status: sum(row.get('status') == status for row in rows) for status in QUEUE_STATES}
        allowed = ', '.join((data.get('cell') or {}).get('sort_classes') or [])
        self.queue_summary.setText('队列 {} 个  ·  可领取 {}  ·  执行中 {}  ·  已完成 {}  |  允许类别：{}'.format(
            len(rows), counts['ready'], counts['reserved'], counts['done'], allowed or '等待读取'))
        if rows is not self._rows:
            self._rows = rows
            self.table.setRowCount(len(rows))
            for i, row in enumerate(rows):
                values = [str(row['id']), row['category'], QUEUE_STATES.get(row.get('status'), '未知')]
                values += ['{:.3f}'.format(v) for v in row['position']]
                values += ['{:.1f}'.format(math.degrees(row['grasp_angle'])) if row.get('grasp_angle') is not None else '—',
                           '{:.1f}'.format(row['grasp_width'] * 1000) if row.get('grasp_width') is not None else '—']
                values += ['{:.2f}'.format(float(row['detection_confidence'])) if 'detection_confidence' in row else '—',
                           str(row.get('observations', '')), '{:.2f}'.format(float(row.get('age', 0.)))]
                for j, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    if row.get('status') == 'reserved':
                        item.setBackground(QColor('#d9f3ed'))
                    self.table.setItem(i, j, item)
