import argparse

import yaml
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QFileDialog
from rqt_gui_py.plugin import Plugin

from .servo_bridge import ServoBridge
from .servo_model import PARAMETERS, add_servo_arguments
from .servo_widget import ServoWidget


class ServoPlugin(Plugin):
    def __init__(self, context):
        super(ServoPlugin, self).__init__(context)
        self.setObjectName('AuboVisualServo')
        self._closed = False
        parser = argparse.ArgumentParser()
        add_servo_arguments(parser)
        args, _ = parser.parse_known_args(context.argv())
        self._widget = ServoWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{} ({})'.format(self._widget.windowTitle(), context.serial_number()))
        context.add_widget(self._widget)
        self._bridge = ServoBridge(args.controller_node, args.servo_ns, args.image_topic, args.geometry_node)
        self._bridge.event.connect(self._event)
        self._widget.command_requested.connect(self._bridge.command)
        self._widget.parameters_requested.connect(self._bridge.apply)
        self._widget.class_requested.connect(self._bridge.select_class)
        self._widget.refresh_requested.connect(self._bridge.refresh)
        self._widget.export_requested.connect(self._export)
        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._tick)
        self._timer.start(200)

    def _tick(self):
        if not self._closed:
            self._widget.render(self._bridge.snapshot())

    def _event(self, kind, value):
        if self._closed: return
        if kind in ('configuration', 'refresh'):
            self._widget.configuration(value, force=kind == 'refresh')
        elif kind == 'applied':
            # Re-read actual values, but retain edits from the other parameter group.
            accepted = value['accepted']
            self._widget.configuration(accepted)
            for key in list(self._widget._dirty):
                if key in self._widget.values() and key in accepted and abs(float(self._widget.values()[key])-float(accepted[key])) < 1e-6:
                    self._widget._dirty.discard(key)
            message = ('服务器拒绝或修正：'+', '.join(value['rejected'])) if value['rejected'] else '参数已应用，已回读服务器实际值。'
            self._widget.parameter_note.setText(message)
            self._widget.log(message)
        elif kind == 'log':
            self._widget.log(value)
        self._tick()

    def _export(self):
        config = self._bridge.snapshot().get('config', {})
        if not config:
            self._widget.log('尚无服务器实际参数可导出')
            return
        path, _ = QFileDialog.getSaveFileName(self._widget, '导出服务器实际参数',
                                             'visual_servo_parameters.yaml', 'YAML (*.yaml *.yml)')
        if path:
            try:
                keys = [p[0] for p in PARAMETERS]+['use_orientation_control']
                with open(path, 'w') as stream:
                    yaml.safe_dump({k: config[k] for k in keys if k in config}, stream, allow_unicode=True)
                self._widget.log('已导出实际参数：'+path+'；重启默认值需另行写入启动配置')
            except OSError as error:
                self._widget.log('导出失败：'+str(error))

    def shutdown_plugin(self):
        self._closed = True
        self._timer.stop()
        self._bridge.close()
