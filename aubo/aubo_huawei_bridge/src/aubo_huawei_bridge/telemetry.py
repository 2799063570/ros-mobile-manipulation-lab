"""Huawei target geometry, bounded by the existing report/string limits."""
import math
import threading
from aubo_cloud_bridge.protocol import Telemetry, decode_json, encode_json


class HuaweiTelemetry(Telemetry):
    def __init__(self):
        super().__init__()
        self.lock = threading.RLock()
        self.values['completed_counts'] = None

    def update(self, kind, raw, now=None):
        with self.lock:
            self._update(kind, raw, now)

    def _update(self, kind, raw, now):
        if kind == 'completed_counts':
            if len(raw.encode('utf-8')) > 64000:
                raise ValueError('completion statistics too large')
            data = decode_json(raw)
            counts = data.get('counts') if isinstance(data, dict) else None
            if (not isinstance(counts, dict) or len(counts) > 200 or
                    any(not isinstance(k, str) or not k or len(k) > 256 or type(v) is not int or
                        not 0 <= v <= 9007199254740991 for k, v in counts.items()) or
                    data.get('scope') != 'node_session' or not isinstance(data.get('session'), str) or
                    not 0 < len(data['session']) <= 128 or data.get('schema_version') != 1 or
                    type(data.get('total')) is not int or data['total'] != sum(counts.values())):
                raise ValueError('invalid completion statistics')
            self.values['completed_counts'] = {key: data[key] for key in ('counts', 'total', 'session', 'scope')}
            return
        super().update(kind, raw, now)
        if kind != 'target_cache':
            return
        targets = list(decode_json(raw)['targets'].values())
        with self.lock:
            rows = decode_json(self.values['target_cache'])['targets']
            for row, target in zip(rows, targets):
                position = target.get('position')
                if (isinstance(position, list) and len(position) == 3 and
                        all(type(v) in (int, float) and math.isfinite(v) for v in position)):
                    row['position'] = [round(v, 4) for v in position]
                for key in ('grasp_angle', 'grasp_width', 'age'):
                    value = target.get(key)
                    if type(value) in (int, float) and math.isfinite(value):
                        row[key] = round(value, 4)
                if type(target.get('observations')) is int:
                    row['observations'] = max(0, min(1000000, target['observations']))
                row['grasp_geometry_valid'] = target.get('grasp_geometry_valid') is True
            self.values['target_cache'] = encode_json({'targets': rows})
