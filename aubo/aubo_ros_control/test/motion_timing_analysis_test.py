#!/usr/bin/env python3
"""Fixture checks: missing returns, skipped hold points, provenance and MAC ages."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("timing", Path(__file__).resolve().parents[1] / "scripts/analyze_motion_timing.py")
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


class AnalysisTest(unittest.TestCase):
    def test_lifecycle_and_missing_events(self):
        columns = "kind id batch_id target_id command_id query_id begin_ns locked_ns end_ns source_ros_ns count mac_scalars result".split()
        columns += [f"{prefix}{i}" for prefix in ("q", "v", "desired") for i in range(6)]
        events = []
        def event(kind, **fields):
            row = dict.fromkeys(columns, 0)
            row.update(kind=kind, mac_scalars=-1)
            row.update(fields)
            events.append(row)
        event("command", command_id=7, begin_ns=100, locked_ns=110, end_ns=120)
        event("target", target_id=5, command_id=7, begin_ns=200, locked_ns=210, end_ns=220)
        event("query_begin", query_id=1, begin_ns=230)
        event("query_end", query_id=1, begin_ns=230, end_ns=240, mac_scalars=240)
        event("point", id=1, batch_id=1, target_id=5, command_id=8, begin_ns=250, locked_ns=260, end_ns=270)
        event("batch_append", id=1, batch_id=1, begin_ns=280, count=1)
        event("send_begin", id=1, batch_id=1, query_id=1, begin_ns=300, count=1, mac_scalars=240)
        event("send_end", id=1, batch_id=1, query_id=1, begin_ns=300, end_ns=400, count=1, mac_scalars=240)
        event("point", id=2, batch_id=2, begin_ns=410, locked_ns=420, end_ns=430)  # not appended
        event("query_begin", query_id=2, begin_ns=450)  # no reply, not MAC zero
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            trace = root / "trace.csv"
            with trace.open("w") as stream:
                stream.write("# motion_timing_trace_v1; dropped=0\n")
                writer = csv.DictWriter(stream, columns)
                writer.writeheader(); writer.writerows(events)
            summary, *_ = timing.analyze(trace, root / "out")
            with (root / "out/point_lifecycle.csv").open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(rows[0]["target_command_id"], "7")
            self.assertEqual(rows[0]["latest_command_id"], "8")
            self.assertEqual(rows[0]["queue_enqueue_ns"], "")
            self.assertEqual(rows[0]["state"], "returned")
            self.assertEqual(rows[1]["state"], "not_sent")
            self.assertEqual(summary["unfinished_queries"], 1)
            self.assertEqual(summary["send_rpc_ms"]["max"], .0001)
            self.assertEqual(summary["local_batch_wait_ms"]["max"], .00002)
            self.assertEqual(summary["observed_mac_zero_after_first_send"], 0)
            self.assertEqual(summary["observed_nonzero_mac_coverage_ms"], {"n": 0})
            # A mismatched point count must not silently yield convincing plots.
            bad = [dict(e) for e in events]
            next(e for e in bad if e['kind'] == 'send_begin')['count'] = 2
            with self.assertRaisesRegex(ValueError, 'point count'):
                timing.validate_events(bad)
            bad = [dict(e) for e in events]
            next(e for e in bad if e['kind'] == 'batch_append')['begin_ns'] = 310
            with self.assertRaisesRegex(ValueError, 'after its SDK send'):
                timing.validate_events(bad)
            with self.assertRaisesRegex(ValueError, 'Duplicate point'):
                timing.validate_events(events + [dict(next(e for e in events if e['kind'] == 'point'))])
            trace.write_text("# capture_incomplete\n")
            with self.assertRaises(ValueError): timing.analyze(trace, root / "out")


if __name__ == "__main__":
    unittest.main()
