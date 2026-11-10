"""Generate import files without tenant IDs, tokens or cloud-side mutations."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ALIAS = 'a1b0c100-0000-4000-8000-000000000001'
WIDGET = 'a1b0c100-0000-4000-8000-000000000002'
KEYS = ('bridge_wall_ms bridge_session commands_enabled sorting_available state state_detail phase description '
        'failure detection_summary target_cache target_cache_age_s queue_total queue_candidate queue_ready '
        'queue_reserved queue_done queue_review last_command completed_counts completed_total '
        'statistics_available workcell').split()


def build():
    config = dict(title='智能分拣工作台', showTitle=False, padding='0px',
                  backgroundColor='#f2f5f9', color='#17263a', dropShadow=False,
                  enableFullscreen=True, settings={}, actions={},
                  datasources=[{'type': 'entity', 'entityAliasId': ALIAS,
                                'dataKeys': [{'name': key, 'label': key, 'type': 'timeseries',
                                              'settings': {}, 'color': '#195ed8'} for key in KEYS]}])
    widget_type = dict(fqn='aubo_sorting_console', name='AUBO Sorting Console', deprecated=False,
                       description='Responsive sorting telemetry and expiring task commands.',
                       descriptor=dict(type='latest', sizeX=24, sizeY=24, resources=[],
                                       templateHtml=(ROOT / 'console.html').read_text(encoding='utf-8'),
                                       templateCss=(ROOT / 'console.css').read_text(encoding='utf-8'),
                                       controllerScript=(ROOT / 'console.js').read_text(encoding='utf-8'),
                                       settingsForm=[], dataKeySettingsForm=[],
                                       defaultConfig=json.dumps(config)))
    bundle = {'widgetsBundle': {'alias': 'aubo_sorting', 'title': 'AUBO Sorting'},
              'widgetTypes': [widget_type]}
    layout = {'widgets': {WIDGET: {'sizeX': 24, 'sizeY': 26, 'row': 0, 'col': 0}},
              'gridSettings': {'columns': 24, 'margin': 0, 'backgroundColor': '#f2f5f9',
                               'mobileAutoFillHeight': True, 'mobileRowHeight': 70}}
    dashboard = {'title': 'AUBO 智能分拣云工作台', 'configuration': {
        'description': 'Import AUBO widget bundle first, then bind alias to one device.',
        'widgets': {WIDGET: {'id': WIDGET, 'typeFullFqn': 'tenant.aubo_sorting_console',
                            'type': 'latest', 'sizeX': 24, 'sizeY': 26, 'row': 0, 'col': 0, 'config': config}},
        'entityAliases': {ALIAS: {'id': ALIAS, 'alias': 'AUBO 分拣设备',
                                'filter': {'type': 'deviceType', 'resolveMultiple': False,
                                           'deviceType': 'aubo_sorting', 'deviceNameFilter': ''}}},
        'states': {'default': {'name': '分拣工作台', 'root': True, 'layouts': {'main': layout}}},
        'settings': {'stateControllerId': 'entity', 'showTitle': False, 'showDashboardTimewindow': False},
        'timewindow': {'realtime': {'timewindowMs': 60000}, 'aggregation': {'type': 'NONE', 'limit': 200}}}}
    for name, value in [('aubo_widgets_bundle.json', bundle), ('aubo_sorting_dashboard.json', dashboard)]:
        (ROOT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    build()
