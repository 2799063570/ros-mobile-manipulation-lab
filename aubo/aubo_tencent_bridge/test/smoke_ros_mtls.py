"""Exercise the real ROS launch and Paho over local mutual TLS; no account or robot."""
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
    result = b''
    while len(result) < size:
        chunk = stream.recv(size - len(result))
        if not chunk:
            raise EOFError()
        result += chunk
    return result


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
    raise ValueError('invalid MQTT length')


def packet(header, payload):
    remaining, length = len(payload), b''
    while True:
        digit = remaining % 128
        remaining //= 128
        length += bytes([digit | (128 if remaining else 0)])
        if not remaining:
            return bytes([header]) + length + payload


def free_port():
    with socket.socket() as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def main():
    with tempfile.TemporaryDirectory(prefix='aubo-tencent-mtls-') as tmp:
        root = Path(tmp)
        cert, key = root / 'local.crt', root / 'local.key'
        subprocess.run(['/usr/bin/openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
            '-keyout', str(key), '-out', str(cert), '-subj', '/CN=localhost',
            '-addext', 'subjectAltName=DNS:localhost',
            '-addext', 'extendedKeyUsage=serverAuth,clientAuth', '-days', '1'],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        key.chmod(0o600)
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.load_cert_chain(str(cert), str(key))
        tls.load_verify_locations(cafile=str(cert))
        tls.verify_mode = ssl.CERT_REQUIRED
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(20)
        mqtt_port = listener.getsockname()[1]
        master_port = free_port()
        received = queue.Queue()
        connected = queue.Queue()
        stopped = threading.Event()

        def broker():
            try:
                with listener.accept()[0] as tcp, tls.wrap_socket(tcp, server_side=True) as stream:
                    stream.settimeout(20)
                    assert stream.getpeercert(), 'client certificate missing'
                    connected.put(stream)
                    while not stopped.is_set():
                        header, payload = read_packet(stream)
                        kind = header >> 4
                        if kind == 1:  # CONNECT
                            stream.sendall(b'\x20\x02\x00\x00')
                        elif kind == 8:  # SUBSCRIBE
                            offset, granted = 2, []
                            while offset < len(payload):
                                size = struct.unpack('!H', payload[offset:offset + 2])[0]
                                offset += 2 + size
                                granted.append(payload[offset])
                                offset += 1
                            stream.sendall(packet(0x90, payload[:2] + bytes(granted)))
                        elif kind == 3:  # PUBLISH; the node uses QoS 0 for reports/replies.
                            assert (header >> 1) & 3 == 0
                            size = struct.unpack('!H', payload[:2])[0]
                            received.put((payload[2:2 + size].decode(), json.loads(payload[2 + size:])))
                        elif kind == 12:
                            stream.sendall(b'\xd0\x00')
                        elif kind == 14:
                            return
            except Exception as error:
                if not stopped.is_set():
                    received.put(error)

        threading.Thread(target=broker, daemon=True).start()
        credentials = root / 'device.json'
        credentials.write_text(json.dumps(dict(product_id='LOCALTEST', device_name='robot', host='localhost',
            ca_file=str(cert), cert_file=str(cert), key_file=str(key))))
        launch = root / 'smoke.launch'
        launch.write_text('''<launch>
  <include file="$(find aubo_tencent_bridge)/launch/tencent.launch">
    <arg name="credentials_file" value="%s"/>
  </include>
  <param name="/aubo_tencent_bridge/port" value="%d"/>
  <param name="/aubo_tencent_bridge/journal_path" value="%s"/>
</launch>''' % (credentials, mqtt_port, root / 'commands.sqlite3'))
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
                        item = received.get(timeout=max(.01, deadline - time.monotonic()))
                        if isinstance(item, Exception):
                            raise item
                        if item[0] == topic:
                            return item[1]
                    raise AssertionError('no MQTT payload for ' + topic)

                report = wait_for('$thing/up/property/LOCALTEST/robot')
                values = report['params']
                assert values['commands_enabled'] == 0
                assert values['sorting_available'] == 0
                assert len(values['bridge_session']) == 32
                assert json.loads(values['bridge_context'])['enabled'] is False
                now = int(time.time() * 1000)
                document = {'method': 'start', 'params': {'command_id': 'a' * 32,
                    'session': values['bridge_session'], 'issued_at_ms': now, 'expires_at_ms': now + 10000}}
                action = {'method': 'action', 'clientToken': 'smoke::1', 'actionId': 'task_command',
                          'params': {'request': json.dumps(document)}}
                topic = '$thing/down/action/LOCALTEST/robot'.encode()
                connected.get(timeout=1).sendall(packet(0x30, struct.pack('!H', len(topic)) + topic + json.dumps(action).encode()))
                reply = wait_for('$thing/up/action/LOCALTEST/robot')
                assert reply['clientToken'] == 'smoke::1'
                assert json.loads(reply['response']['result'])['status'] == 'rejected'
                master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
                code, _, state = master.getSystemState('/tencent_smoke_check')
                assert code == 1
                assert not any(name.startswith('/sorting/') for name, _ in state[2])
                print('PASS: official launch + real system Paho + verified mutual TLS property report')
                print('PASS: real MQTT action reply rejects read-only command; no robot service started')
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
