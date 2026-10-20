"""Bounded pyserial byte-stream adapter. Import pyserial only when a port is opened."""
from .protocol import Parser


class SerialTransport:
    def __init__(self, port, baud=115200, factory=None):
        if factory is None:
            import serial
            factory = serial.Serial
        self.port = factory(port=port, baudrate=baud, timeout=0.01, write_timeout=0.05,
                            xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
        except Exception:
            self.port.close()
            raise
        self.parser = Parser()
        self.received_bytes = 0
        self.received_frames = 0

    def read(self):
        count = max(1, min(256, self.port.in_waiting))
        data = self.port.read(count)
        self.received_bytes += len(data)
        frames = self.parser.feed(data)
        self.received_frames += len(frames)
        return frames

    def write(self, frame):
        data = frame.encode()
        if self.port.write(data) != len(data):
            raise IOError('partial serial write; reconnecting with a new session')

    def close(self):
        self.port.close()
