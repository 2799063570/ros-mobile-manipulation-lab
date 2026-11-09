"""Launch the actual bridge against an isolated ROS master and local TLS MQTT fixture."""
import json
import os
from pathlib import Path
import queue
import signal
import socket
import ssl
import struct
import subprocess
import tempfile
import threading
import time
import xmlrpc.client


def exact(stream, size):
    data = b''
    while len(data) < size:
        chunk = stream.recv(size - len(data))
        if not chunk:
            raise EOFError()
        data += chunk
    return data


def read_packet(stream):
    header = exact(stream, 1)[0]
    length, multiplier = 0, 1
    for _ in range(4):
        digit = exact(stream, 1)[0]
        length += (digit & 127) * multiplier
        if not digit & 128:
            assert length <= 65536
            return header, exact(stream, length)
        multiplier *= 128
    raise ValueError('bad MQTT length')


def packet(header, data):
    size, encoded = len(data), b''
    while True:
        digit = size % 128
        size //= 128
        encoded += bytes([digit | (128 if size else 0)])
        if not size:
            return bytes([header]) + encoded + data


def free_port():
    with socket.socket() as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def main():
    with tempfile.TemporaryDirectory(prefix='aubo-huawei-tls-') as tmp:
        root = Path(tmp)
        cert, key = root / 'local.crt', root / 'local.key'
        subprocess.run(['/usr/bin/openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
            '-keyout', str(key), '-out', str(cert), '-subj', '/CN=localhost',
            '-addext', 'subjectAltName=DNS:localhost', '-days', '1'],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.load_cert_chain(str(cert), str(key))
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(20)
        port = listener.getsockname()[1]
        master_port = free_port()
        events, connections = queue.Queue(), queue.Queue()
        stopped = threading.Event()

        def broker():
            try:
                with listener.accept()[0] as tcp, tls.wrap_socket(tcp, server_side=True) as stream:
                    stream.settimeout(20)
                    connections.put(stream)
                    while not stopped.is_set():
                        header, data = read_packet(stream)
                        kind = header >> 4
                        if kind == 1:
                            # MQTT 3.1.1 CONNECT has a ten-byte variable header.
                            offset, fields = 10, []
                            for _ in range(3):
                                size = struct.unpack('!H', data[offset:offset + 2])[0]
                                fields.append(data[offset + 2:offset + 2 + size].decode())
                                offset += size + 2
                            assert fields[0].startswith('LOCALTEST_robot_0_0_')
                            assert fields[1] == 'LOCALTEST_robot'
                            assert len(fields[2]) == 64 and fields[2] != 'local-test-secret'
                            stream.sendall(b'\x20\x02\x00\x00')
                        elif kind == 8:
                            stream.sendall(packet(0x90, data[:2] + b'\x01'))
                        elif kind == 3:
                            assert (header >> 1) & 3 == 0
                            size = struct.unpack('!H', data[:2])[0]
                            events.put((data[2:2 + size].decode(), json.loads(data[2 + size:])))
                        elif kind == 12:
                            stream.sendall(b'\xd0\x00')
                        elif kind == 14:
                            return
            except Exception as error:
                if not stopped.is_set():
                    events.put(error)

        threading.Thread(target=broker, daemon=True).start()
        config = root / 'device.json'
        config.write_text(json.dumps(dict(device_id='LOCALTEST_robot', device_secret='local-test-secret',
                                          host='localhost', ca_file=str(cert))))
        config.chmod(0o600)
        launch = root / 'smoke.launch'
        launch.write_text('''<launch>
  <include file="$(find aubo_huawei_bridge)/launch/huawei.launch">
    <arg name="credentials_file" value="%s"/>
  </include>
  <param name="/aubo_huawei_bridge/port" value="%d"/>
  <param name="/aubo_huawei_bridge/journal_path" value="%s"/>
  <param name="/aubo_huawei_bridge/budget_path" value="%s"/>
</launch>''' % (config, port, root / 'commands.sqlite3', root / 'budget.sqlite3'))
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % master_port,
                   ROS_HOSTNAME='localhost', ROS_LOG_DIR=str(root / 'ros-logs'))
        env.pop('ROS_IP', None)
        output = root / 'launch.log'
        with output.open('w') as log:
            process = subprocess.Popen(['roslaunch', '-p', str(master_port), str(launch)],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                def wait_for(topic):
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        value = events.get(timeout=max(.01, deadline - time.monotonic()))
                        if isinstance(value, Exception):
                            raise value
                        if value[0] == topic:
                            return value[1]
                    raise AssertionError('missing MQTT message')

                report = wait_for('$oc/devices/LOCALTEST_robot/sys/properties/report')
                assert len(json.dumps(report, ensure_ascii=False, separators=(',', ':')).encode()) <= 3800
                values = report['services'][0]['properties']
                assert values['commands_enabled'] == 0
                assert values['sorting_available'] == 0
                cli = subprocess.run(['rosrun', 'aubo_huawei_bridge', 'huawei_make_command', 'start', '--paras-only'],
                    env=env, capture_output=True, text=True, timeout=10, check=True)
                params = json.loads(cli.stdout)
                assert params['session'] == values['bridge_session']
                command = {'service_id': 'Sorting', 'command_name': 'task_command', 'paras': params}
                topic = '$oc/devices/LOCALTEST_robot/sys/commands/request_id=smoke-request-1'.encode()
                connections.get(timeout=1).sendall(packet(0x30,
                    struct.pack('!H', len(topic)) + topic + json.dumps(command).encode()))
                reply = wait_for('$oc/devices/LOCALTEST_robot/sys/commands/response/request_id=smoke-request-1')
                assert reply['result_code'] == 1
                assert json.loads(reply['paras']['result'])['status'] == 'rejected'
                master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
                code, _, state = master.getSystemState('/huawei_smoke_check')
                assert code == 1
                assert not any(name.startswith('/sorting/') for name, _ in state[2])
                print('PASS: real ROS launch, system Paho, TLS certificate verification and Huawei MQTT authentication')
                print('PASS: bounded property report, live-session CLI and read-only MQTT command response')
            except Exception:
                print(output.read_text())
                raise
            finally:
                stopped.set()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                listener.close()


if __name__ == '__main__':
    main()
