"""Projection and node-session production counts for the ThingsBoard workstation."""
import math
import threading
from .protocol import Telemetry, decode_json, encode_json


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class SortingTelemetry(Telemetry):
    def __init__(self):
        super().__init__()
        self.lock = threading.RLock()
        self.values.update(completed_counts='null', completed_total=0,
                           statistics_available=False, workcell='{}')

    def set_workcell(self, center, size, frame):
        valid = (isinstance(center, list) and len(center) >= 2 and
                 isinstance(size, list) and len(size) >= 2 and
                 all(finite(v) for v in center[:2] + size[:2]) and
                 all(0 < v <= 100 for v in size[:2]) and
                 isinstance(frame, str) and 0 < len(frame) <= 64)
        cell = {'table_center': center[:2], 'table_size': size[:2], 'frame': frame} if valid else {}
        with self.lock:
            self.values['workcell'] = encode_json(cell)

    def update(self, kind, raw, now=None):
        with self.lock:
            if kind == 'completed_counts':
                if len(raw.encode('utf-8')) > 64000:
                    raise ValueError('completion statistics too large')
                data = decode_json(raw)
                counts = data.get('counts') if isinstance(data, dict) else None
                if (not isinstance(counts, dict) or len(counts) > 200 or
                        any(not isinstance(k, str) or not 0 < len(k) <= 256 or
                            type(v) is not int or not 0 <= v <= 9007199254740991
                            for k, v in counts.items()) or
                        data.get('schema_version') != 1 or data.get('scope') != 'node_session' or
                        not isinstance(data.get('session'), str) or not 0 < len(data['session']) <= 128 or
                        type(data.get('total')) is not int or not 0 <= data['total'] <= 9007199254740991 or
                        data['total'] != sum(counts.values())):
                    raise ValueError('invalid completion statistics')
                stats = {key: data[key] for key in ('counts', 'total', 'scope', 'session')}
                self.values.update(completed_counts=encode_json(stats), completed_total=data['total'],
                                   statistics_available=True)
                return
            super().update(kind, raw, now)
            if kind != 'target_cache':
                return
            rows = decode_json(self.values['target_cache'])['targets']
            targets = decode_json(raw)['targets'].values()
            for row, target in zip(rows, targets):
                position = target.get('position')
                if isinstance(position, list) and len(position) == 3 and all(finite(v) for v in position):
                    row['position'] = [round(v, 4) for v in position]
                for key in ('grasp_angle', 'grasp_width', 'age'):
                    if finite(target.get(key)):
                        row[key] = round(target[key], 4)
                if type(target.get('observations')) is int:
                    row['observations'] = max(0, min(1000000, target['observations']))
                row['grasp_geometry_valid'] = target.get('grasp_geometry_valid') is True
            self.values['target_cache'] = encode_json({'targets': rows})
