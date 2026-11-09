"""Same-origin browser API. Runs locally or as FunctionGraph index.handler.

No ROS dependency in the cloud. Application IAM credentials never enter the page.
"""
import base64
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import time
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
METHODS = {'move_to_observation', 'start', 'stop', 'home'}
IDLE = {'IDLE', 'READY', 'STOPPED', 'ERROR'}


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


class APIError(Exception):
    def __init__(self, status, message):
        self.status, self.message = status, message


class Cloud:
    def __init__(self):
        from huaweicloudsdkcore.auth.credentials import BasicCredentials
        from huaweicloudsdkcore.http.http_config import HttpConfig
        from huaweicloudsdkiotda.v5 import IoTDAClient
        endpoint = os.environ['IOTDA_ENDPOINT'].rstrip('/')
        if not re.fullmatch(r'https://[a-zA-Z0-9.-]+\.myhuaweicloud\.com', endpoint):
            raise ValueError('invalid IOTDA_ENDPOINT')
        config = HttpConfig.get_default_config()
        config.timeout = (3, 12)
        # Never retry a task after an ambiguous timeout.
        config.retry_times = 0
        creds = BasicCredentials(os.environ['HUAWEICLOUD_SDK_AK'],
                                 os.environ['HUAWEICLOUD_SDK_SK'], os.environ['IOTDA_PROJECT_ID'])
        self.client = IoTDAClient.new_builder().with_credentials(creds).with_http_config(
            config).with_endpoint(endpoint).build()
        self.device = os.environ['IOTDA_DEVICE_ID']

    def snapshot(self):
        from huaweicloudsdkiotda.v5 import ShowDeviceShadowRequest
        reply = self.client.show_device_shadow(ShowDeviceShadowRequest(device_id=self.device)).to_dict()
        props = next((s.get('reported', {}).get('properties', {}) for s in reply.get('shadow', [])
                      if s.get('service_id') == 'Sorting'), {})
        return props

    def command(self, body):
        from huaweicloudsdkiotda.v5 import CreateCommandRequest, DeviceCommandRequest
        return self.client.create_command(CreateCommandRequest(device_id=self.device,
            body=DeviceCommandRequest(**body))).to_dict()


class Demo:
    """Explicit fixture provider: cannot dispatch any commands."""
    def snapshot(self):
        return {'state': 'READY', 'state_detail': '演示数据 · 等待队列分拣', 'phase': 'DETECTING',
                'description': '目标已确认，等待操作', 'failure': '',
                'bridge_wall_ms': str(int(time.time() * 1000)), 'bridge_session': 'a' * 32,
                'commands_enabled': 0, 'sorting_available': 1, 'target_cache_age_s': 1.2,
                'detection_summary': 'bottle:2 can:1 box:0', 'queue_total': 3,
                'queue_candidate': 1, 'queue_ready': 2, 'queue_reserved': 0,
                'queue_done': 0, 'queue_review': 0, 'target_cache_truncated': 0,
                'target_cache': dumps({'targets': [
                    {'id': i + 1, 'category': cat, 'status': status, 'position': pos,
                     'detection_confidence': conf, 'observations': 5 + i, 'age': 1.2,
                     'grasp_angle': 0, 'grasp_width': .04, 'grasp_geometry_valid': True}
                    for i, (cat, status, pos, conf) in enumerate([
                        ('bottle', 'ready', [-.22, -.52, .04], .96),
                        ('can', 'ready', [.08, -.71, .03], .93),
                        ('bottle', 'candidate', [-.13, -.9, .04], .87)])]}), 'last_command': '{}'}

    def command(self, body):
        raise APIError(403, '演示模式不能发送任务')


class App:
    def __init__(self, provider=None, clock=time.time):
        self.clock = clock
        self.mode = os.getenv('AUBO_DASHBOARD_MODE', 'cloud')
        if self.mode not in ('cloud', 'demo', 'ros') or (self.mode == 'ros' and provider is None):
            raise ValueError('ROS mode requires an explicit local provider')
        self.demo = self.mode == 'demo'
        self.provider = provider
        self.password = os.getenv('DASHBOARD_PASSWORD', '')
        self.key = os.getenv('DASHBOARD_SESSION_KEY', '').encode()
        if self.demo:
            self.password = self.password or 'demo'
            self.key = self.key or b'local-preview-only-do-not-deploy'
        if not self.password or len(self.key) < 32:
            raise ValueError('configure DASHBOARD_PASSWORD and DASHBOARD_SESSION_KEY (32+ chars)')
        if not self.demo and len(self.password) < 20:
            raise ValueError('use a random DASHBOARD_PASSWORD of at least 20 chars')
        self.controls = not self.demo and os.getenv('DASHBOARD_ENABLE_COMMANDS') == 'true'

    def sign(self, purpose, payload):
        data = base64.urlsafe_b64encode(dumps(payload).encode()).decode().rstrip('=')
        sig = hmac.new(self.key, (purpose + ':' + data).encode(), hashlib.sha256).hexdigest()
        return data + '.' + sig

    def verify(self, purpose, token):
        try:
            data, signature = token.split('.')
            expected = hmac.new(self.key, (purpose + ':' + data).encode(), hashlib.sha256).hexdigest()
            if not hmac.compare_digest(expected, signature):
                raise ValueError()
            payload = json.loads(base64.urlsafe_b64decode(data + '=' * (-len(data) % 4)))
            if payload['exp'] <= self.clock():
                raise ValueError()
            return payload
        except (ValueError, KeyError, TypeError):
            raise APIError(401, '会话或操作已过期，请重新登录或核对状态')

    def source(self):
        if self.provider is None:
            self.provider = Demo() if self.demo else Cloud()
        return self.provider

    def snapshot(self):
        props = self.source().snapshot()
        try:
            age = (self.clock() * 1000 - int(props['bridge_wall_ms'])) / 1000
            fresh = 0 <= age <= getattr(self.source(), 'fresh_seconds', 90) and bool(re.fullmatch(r'[a-f0-9]{32}', props['bridge_session']))
        except (ValueError, KeyError, TypeError):
            age, fresh = None, False
        return {'properties': props, 'server_ms': int(self.clock() * 1000), 'age_s': age,
                'fresh': fresh, 'mode': self.mode,
                'camera_enabled': bool(getattr(self.source(), 'camera_enabled', False)),
                'workcell': self.source().workcell() if hasattr(self.source(), 'workcell') else {},
                'controls_enabled': self.controls, 'poll_seconds': getattr(self.source(), 'poll_seconds', 30)}

    def guard(self, snapshot, method):
        p = snapshot['properties']
        if not self.controls or not snapshot['fresh'] or p.get('commands_enabled') != 1 or p.get('sorting_available') != 1:
            raise APIError(409, '当前只读、数据过期或分拣接口不可用')
        if method not in METHODS:
            raise APIError(400, '不支持的任务')
        if method != 'stop' and (p.get('state') not in IDLE or (method == 'start' and p.get('state') != 'READY')):
            raise APIError(409, '设备状态已变化，请刷新后确认')

    def route(self, method, path, headers=None, body=b''):
        headers = {k.lower(): v for k, v in (headers or {}).items()}
        path = urlsplit(path).path
        status, extra = 200, {}
        try:
            if method == 'GET' and path in ('/', '/index.html', '/app.js', '/style.css', '/manifest.webmanifest', '/icon.svg'):
                name = 'index.html' if path == '/' else path[1:]
                types = {'.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css',
                         '.webmanifest': 'application/manifest+json', '.svg': 'image/svg+xml'}
                content = (ROOT / 'dist' / name).read_bytes()
                extra['Content-Type'] = types[Path(name).suffix] + '; charset=utf-8'
            elif path.startswith('/api/'):
                if len(body) > 8192:
                    raise APIError(413, '请求过大')
                data = {}
                if method == 'POST':
                    if headers.get('content-type', '').split(';')[0] != 'application/json':
                        raise APIError(415, '需要 JSON 请求')
                    data = json.loads(body or b'{}')
                    if not isinstance(data, dict):
                        raise APIError(400, '无效的请求')
                if method == 'POST' and path == '/api/login':
                    supplied = str(data.get('password', ''))
                    if not hmac.compare_digest(hashlib.sha256(supplied.encode()).digest(),
                                               hashlib.sha256(self.password.encode()).digest()):
                        raise APIError(401, '访问口令错误')
                    result = {'token': self.sign('login', {'exp': self.clock() + 7200,
                                                         'nonce': secrets.token_hex(16)})}
                else:
                    self.verify('login', headers.get('authorization', '')[7:] if headers.get('authorization', '').startswith('Bearer ') else '')
                    if method == 'GET' and path == '/api/camera':
                        source = self.source()
                        frame = source.camera() if getattr(source, 'camera_enabled', False) else None
                        if frame is None:
                            raise APIError(503, '识别画面尚未就绪或已中断')
                        content, age, sequence = frame
                        extra.update({'Content-Type': 'image/jpeg', 'X-Camera-Age': str(round(age, 2)),
                                      'X-Camera-Sequence': str(sequence)})
                        return 200, self.headers(extra), content
                    if method == 'GET' and path == '/api/snapshot':
                        result = self.snapshot()
                    elif method == 'POST' and path == '/api/prepare':
                        snapshot = self.snapshot()
                        action = data.get('method')
                        self.guard(snapshot, action)
                        now = int(self.clock() * 1000)
                        envelope = {'service_id': 'Sorting', 'command_name': 'task_command', 'paras': {
                            'method': action, 'command_id': secrets.token_hex(16),
                            'session': snapshot['properties']['bridge_session'],
                            'issued_at_ms': str(now), 'expires_at_ms': str(now + 25000)}}
                        result = {'ticket': self.sign('command', {'exp': self.clock() + 25, 'body': envelope})}
                    elif method == 'POST' and path == '/api/command':
                        envelope = self.verify('command', data.get('ticket', ''))['body']
                        snapshot = self.snapshot()
                        self.guard(snapshot, envelope['paras']['method'])
                        if envelope['paras']['session'] != snapshot['properties']['bridge_session']:
                            raise APIError(409, '设备会话已变化，请重新核对状态')
                        # SDK retries disabled; same ticket keeps the same persistent ROS command ID.
                        try:
                            response = self.source().command(envelope)
                            result = {'status': 'response', 'response': response}
                        except Exception:
                            result = {'status': 'unknown', 'command_id': envelope['paras']['command_id'],
                                      'message': '结果未知。请核对状态和过程记录，不要再次发起同一动作。'}
                    else:
                        raise APIError(404, '接口不存在')
                content = dumps(result).encode()
                extra['Content-Type'] = 'application/json; charset=utf-8'
            else:
                raise APIError(404, '页面不存在')
        except APIError as exc:
            status, content = exc.status, dumps({'error': exc.message}).encode()
            extra['Content-Type'] = 'application/json; charset=utf-8'
        except (ValueError, TypeError):
            status, content = 400, dumps({'error': '请求格式错误'}).encode()
        except Exception:
            # Do not leak SDK exceptions, tokens, environment, or provider headers.
            status, content = 502, dumps({'error': '设备读取失败，请检查连接及后台配置'}).encode()
        return status, self.headers(extra), content

    def headers(self, extra):
        extra.setdefault('Content-Type', 'application/json; charset=utf-8')
        extra.update({'Cache-Control': 'no-store', 'X-Aubo-Workstation': self.mode, 'X-Content-Type-Options': 'nosniff',
                      'Referrer-Policy': 'no-referrer', 'X-Frame-Options': 'DENY',
                      'Content-Security-Policy': "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'"})
        return extra


_application = None


def handler(event, context):
    """FunctionGraph APIG event adapter; also accepts the v2 HTTP request shape."""
    global _application
    if _application is None:
        _application = App()
    if isinstance(event, str):
        event = json.loads(event)
    method = event.get('httpMethod') or event.get('requestContext', {}).get('http', {}).get('method', 'GET')
    body = event.get('body') or ''
    raw = base64.b64decode(body) if event.get('isBase64Encoded') else body.encode()
    status, headers, content = _application.route(method, event.get('rawPath') or event.get('path', '/'),
                                                  event.get('headers'), raw)
    binary = headers.get('Content-Type') == 'image/jpeg'
    return {'statusCode': status, 'headers': headers,
            'body': base64.b64encode(content).decode() if binary else content.decode(), 'isBase64Encoded': binary}


if __name__ == '__main__':
    import argparse
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--port', type=int, default=8099)
    args = parser.parse_args()
    if args.demo:
        os.environ['AUBO_DASHBOARD_MODE'] = 'demo'
    application = App()

    class HTTPHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.respond()

        def do_POST(self):
            self.respond()

        def respond(self):
            try:
                length = int(self.headers.get('Content-Length', 0))
            except ValueError:
                length = 8193
            if length < 0 or length > 8192:
                self.send_error(413)
                return
            status, headers, content = application.route(self.command, self.path, dict(self.headers), self.rfile.read(length))
            self.send_response(status)
            for name, value in headers.items():
                self.send_header(name, value)
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), HTTPHandler)
    print('AUBO dashboard: http://127.0.0.1:%s · %s' % (args.port, '演示 / 口令 demo' if args.demo else '云端数据'), flush=True)
    server.serve_forever()
