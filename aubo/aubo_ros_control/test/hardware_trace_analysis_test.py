#!/usr/bin/env python3
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('hardware_analysis', Path(__file__).resolve().parents[1] / 'scripts/analyze_hardware_trace.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class HardwareTraceAnalysisTest(unittest.TestCase):
    def capture(self, root, received_age=1, retransmissions=0, hint=12345, kind='send'):
        prefix = Path(root)/'test'
        with Path(str(prefix)+'.commands.csv').open('w') as out:
            out.write('# hardware_commands_v1; period_s=0.005; dropped=0\n')
            fields=['id','kind','steady_ns','ros_ns','sample_ns']+['q'+str(i) for i in range(6)]
            writer=csv.DictWriter(out,fields);writer.writeheader()
            for ident in [1,2]:
                row=dict(id=ident,kind='seed' if ident==1 else 'command',steady_ns=ident*5000000,ros_ns=ident*5000000,sample_ns=ident*5000000)
                row.update({f'q{i}':(.4+(ident-1)*1e-6 if i==0 else 0) for i in range(6)})
                writer.writerow(row)
        fields='kind,id,first_point_id,ros_ns,begin_ns,end_ns,count,mac_scalars,result,port_hint,phase,tcp_valid,fd,local_port,bytes_valid,bytes_sent,bytes_acked,bytes_received,retransmissions,rtt_us,rto_us,last_data_sent_ms,last_data_received_ms,unacked,notsent_bytes'.split(',')
        with Path(str(prefix)+'.rpc.csv').open('w') as out:
            out.write('# hardware_rpc_v1; dropped=0\n'); writer=csv.DictWriter(out,fields);writer.writeheader()
            for after in [False,True]:
                row=dict.fromkeys(fields,0)
                row.update(kind=kind,id=1,first_point_id=1,ros_ns=1000000000,begin_ns=1000000000,end_ns=1200000000,count=2,mac_scalars=240,
                           port_hint=hint,phase='after' if after else 'before',tcp_valid=1,fd=5,local_port=12345,bytes_valid=1,
                           bytes_sent=200 if after else 100,bytes_acked=200 if after else 100,bytes_received=70 if after else 50,
                           retransmissions=retransmissions if after else 0,last_data_received_ms=received_age if after else 0,rtt_us=20000,rto_us=200000)
                writer.writerow(row)
        return prefix

    def test_late_payload_and_command_derivatives(self):
        with tempfile.TemporaryDirectory() as root:
            report=analysis.analyze(self.capture(root))
            self.assertAlmostEqual(report['queued_sample_peaks']['jerk_rad_s3'],8.,places=4)
            self.assertEqual(report['command_grid_outliers'],0)
            self.assertEqual(report['latency']['send']['max_ms'],200.)
            self.assertEqual(report['batch_first_point_queue_wait']['max_ms'],995.)
            self.assertEqual(report['slow_calls'][0]['queue_wait']['last_point_queue_wait_ms'],990.)
            self.assertIn('near SDK return',report['slow_calls'][0]['observation'])
            self.assertEqual(report['slow_calls'][0]['socket_deltas'][0]['bytes_sent_delta'],100)

    def test_early_payload_retransmission_and_unknown_socket_remain_distinct(self):
        for age,retrans,hint,expected in [(180,0,12345,'well before'),(1,1,12345,'Outbound retransmission'),(1,0,0,'No comparable')]:
            with tempfile.TemporaryDirectory() as root:
                report=analysis.analyze(self.capture(root,age,retrans,hint))
                self.assertIn(expected,report['slow_calls'][0]['observation'])

    def test_incomplete_capture_and_command_gap_are_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            prefix=self.capture(root);path=Path(str(prefix)+'.commands.csv')
            text=path.read_text();path.write_text(text.replace('2,command','3,command'))
            with self.assertRaises(ValueError):analysis.analyze(prefix)
            path.write_text('# incomplete: waiting for shutdown\n')
            with self.assertRaises(ValueError):analysis.analyze(prefix)

    def test_feedback_latency_uses_its_own_socket_without_command_association(self):
        with tempfile.TemporaryDirectory() as root:
            prefix = self.capture(root, retransmissions=1, kind='feedback_query')
            report = analysis.analyze(prefix)
            self.assertEqual(report['latency']['feedback_query']['max_ms'], 200.)
            self.assertEqual(report['latency']['send']['n'], 0)
            self.assertEqual(report['batch_first_point_queue_wait']['n'], 0)
            event = report['slow_calls'][0]
            self.assertEqual(event['queue_wait'], {})
            self.assertTrue(event['socket_deltas'][0]['is_rpc_login_hint'])
            self.assertFalse(event['socket_deltas'][0]['is_mac_login_hint'])
            self.assertIn('Outbound retransmission', event['observation'])


if __name__=='__main__':unittest.main()
