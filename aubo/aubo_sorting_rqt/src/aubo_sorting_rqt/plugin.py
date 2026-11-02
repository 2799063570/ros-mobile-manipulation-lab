import argparse

import yaml
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QFileDialog
from rqt_gui_py.plugin import Plugin

from .bridge import RosBridge
from .model import ACTIONS, PARAMETERS, STATES, add_plugin_arguments, state_code
from .widget import SortingWidget


class SortingPlugin(Plugin):
    def __init__(self, context):
        super(SortingPlugin, self).__init__(context)
        self.setObjectName('AuboSorting')
        self._closed = False
        parser = argparse.ArgumentParser()
        add_plugin_arguments(parser)
        args, _ = parser.parse_known_args(context.argv())
        self._namespace = args.sorting_ns.rstrip('/')
        self._widget = SortingWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{} ({})'.format(self._widget.windowTitle(), context.serial_number()))
        context.add_widget(self._widget)
        self._bridge = RosBridge(args.task_node, args.sorting_ns, args.image_topic, args.hardware_topic)
        self._bridge.event.connect(self._event)
        self._widget.command_requested.connect(self._command)
        self._widget.parameters_requested.connect(self._bridge.apply)
        self._widget.refresh_requested.connect(self._bridge.refresh)
        self._widget.export_requested.connect(self._export)
        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._tick)
        self._timer.start(200)

    def _tick(self):
        if self._closed:
            return
        data = self._bridge.snapshot()
        data['services'] = {key for key, _ in ACTIONS if self._namespace+'/'+key in data['services']}
        self._widget.render(data)

    def _command(self, name):
        self._widget.log('发送操作：{}'.format(dict(ACTIONS)[name]))
        self._bridge.command(name)

    def _event(self, kind, value):
        if self._closed:
            return
        if kind == 'state':
            self._widget.log('状态：{}  {}'.format(STATES.get(state_code(value), state_code(value)), value))
        elif kind == 'progress':
            from .model import PHASES
            self._widget.log('过程：{}  {}'.format(PHASES.get(value.get('phase'), value.get('phase', '')), value.get('description', '')))
        elif kind == 'command':
            label = dict(ACTIONS).get(value['name'], value['name'])
            self._widget.log('{}：{} · {}'.format(label, '请求已接受，等待任务状态' if value['success'] else '请求被拒绝', value['message']))
            self._widget.notice.setText('{}：{} · {}'.format(label, '请求已接受' if value['success'] else '请求被拒绝', value['message']))
        elif kind in ('configuration', 'refresh'):
            self._widget.configuration(value, force=kind == 'refresh')
        elif kind == 'applied':
            self._widget.configuration(value['accepted'], force=True)
            message = ('服务器未接受部分编辑值：'+', '.join(value['rejected'])) if value['rejected'] else '在线参数已应用，并已读回实际值。'
            self._widget.parameter_note.setText(message)
            self._widget.notice.setText(message)
            self._widget.log(message)
        elif kind == 'error':
            self._widget.log('错误：'+str(value))
            self._widget.parameter_note.setText(str(value))
            self._widget.notice.setText(str(value))
        self._tick()

    def _export(self):
        config = self._bridge.snapshot().get('config', {})
        if not config:
            self._widget.log('尚无服务器实际参数可导出。')
            return
        path, _ = QFileDialog.getSaveFileName(self._widget, '导出服务器实际参数', 'sorting_parameters.yaml', 'YAML (*.yaml *.yml)')
        if path:
            try:
                with open(path, 'w') as stream:
                    yaml.safe_dump({key: config[key] for key, *_ in PARAMETERS if key in config}, stream, allow_unicode=True)
                self._widget.log('已导出服务器实际参数：'+path)
            except OSError as error:
                self._widget.log('导出失败：'+str(error))

    def shutdown_plugin(self):
        self._closed = True
        self._timer.stop()
        self._bridge.close()
