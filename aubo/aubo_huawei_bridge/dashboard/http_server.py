"""Bounded loopback HTTP server for authenticated, temporary cpolar demonstrations."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
import threading


class BoundedServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, address, app):
        self.app = app
        self.slots = threading.BoundedSemaphore(16)
        super().__init__(address, Handler)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(5)
        return connection, address

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    # One request per connection; reject chunked and ambiguous lengths at this boundary.
    protocol_version = 'HTTP/1.0'
    server_version = 'AuboDashboard'
    sys_version = ''

    def do_GET(self):
        self.respond()

    def do_POST(self):
        self.respond()

    def respond(self):
        lengths = self.headers.get_all('Content-Length') or []
        if self.headers.get('Transfer-Encoding') or len(lengths) > 1:
            self.send_error(400)
            return
        try:
            length = int(lengths[0]) if lengths else 0
        except ValueError:
            self.send_error(400)
            return
        if not 0 <= length <= 8192:
            self.send_error(413)
            return
        # Do not accept active requests delivered via the tunnel's plain HTTP URL.
        # Loopback access has no forwarding header. cpolar HTTPS supplies this header.
        forwarded = self.headers.get('X-Forwarded-Proto', '').lower()
        if forwarded and forwarded != 'https':
            self.send_error(403, 'Use the HTTPS tunnel address')
            return
        try:
            body = self.rfile.read(length)
            if len(body) != length:
                self.send_error(400)
                return
            status, headers, content = self.server.app.route(self.command, self.path, dict(self.headers), body)
            self.send_response(status)
            for name, value in headers.items():
                self.send_header(name, value)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(content)
        except (socket.timeout, BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, *args):
        # Avoid logging passwords, bearer tokens or signed command tickets.
        pass
