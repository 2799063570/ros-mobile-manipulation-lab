#!/usr/bin/env python3
"""PC-only serial link test; --demo-feedback explicitly labels synthetic feedback on OLED."""
import argparse
import sys
import time
from pathlib import Path

# Also supports direct execution from the source tree without Catkin or ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aubo_remote_bridge.core import BridgeCore
from aubo_remote_bridge.protocol import SOURCE_NONE, SOURCE_DEMO, feedback_frame
from aubo_remote_bridge.transport import SerialTransport


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='COM5 or /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--demo-feedback', action='store_true', help='return synthetic joints, clearly marked DEMO')
    args = parser.parse_args()
    transport = SerialTransport(args.port, args.baud)
    core = BridgeCore()
    printed, was_online = 0.0, False
    diagnostic_at = time.monotonic() + 2
    accepted = 0
    warned_protocol = False
    print(f'OPEN {args.port}, {args.baud} 8N1; HOME/ZERO v2 protocol (0x01/0x02/0x03). '
          'Diagnostic only; no robot control. Waiting for INPUT replies...', flush=True)
    try:
        while True:
            request = core.request(time.monotonic())
            if request is not None:
                transport.write(request)
            for frame in transport.read():
                if frame.kind in (0x11, 0x12, 0x13) and not warned_protocol:
                    warned_protocol = True
                    print('PROTOCOL MISMATCH: Cartesian firmware requires cartesian_serial_bridge; '
                          'this monitor uses HOME/ZERO protocol.', flush=True)
                now = time.monotonic()
                state = core.receive(frame, now)
                if state is None:
                    continue
                accepted += 1
                source = SOURCE_DEMO if args.demo_feedback else SOURCE_NONE
                joints = (state.x, state.y, 0, 0, 0, 0) if args.demo_feedback else (0,) * 6
                transport.write(feedback_frame(core.session, frame.sequence, source, 63 if args.demo_feedback else 0, joints))
                if now - printed >= 0.2:
                    print(f'RAW=({state.raw_x},{state.raw_y}) AXES={state.axes} SW={state.button} '
                          f'VALID={state.valid} PAGE={state.page} FEEDBACK={"DEMO" if args.demo_feedback else "NONE"}', flush=True)
                    printed = now
            online = core.online(time.monotonic())
            if time.monotonic() >= diagnostic_at:
                diagnostic_at = time.monotonic() + 2
                print(f'RX bytes={transport.received_bytes} frames={transport.received_frames} '
                      f'errors={transport.parser.errors} accepted={accepted} online={online}; '
                      'requires binary INPUT v2, matching session/sequence within 100ms (not ASCII keys)', flush=True)
            if online != was_online:
                print('LINK OK' if online else 'LINK TIMEOUT', flush=True)
                was_online = online
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()


if __name__ == '__main__':
    main()
