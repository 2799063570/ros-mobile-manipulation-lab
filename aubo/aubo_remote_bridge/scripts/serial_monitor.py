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
    try:
        while True:
            request = core.request(time.monotonic())
            if request is not None:
                transport.write(request)
            for frame in transport.read():
                now = time.monotonic()
                state = core.receive(frame, now)
                if state is None:
                    continue
                source = SOURCE_DEMO if args.demo_feedback else SOURCE_NONE
                joints = (state.x, state.y, 0, 0, 0, 0) if args.demo_feedback else (0,) * 6
                transport.write(feedback_frame(core.session, frame.sequence, source, 63 if args.demo_feedback else 0, joints))
                if now - printed >= 0.2:
                    print(f'RAW=({state.raw_x},{state.raw_y}) AXES={state.axes} SW={state.button} '
                          f'VALID={state.valid} PAGE={state.page} FEEDBACK={"DEMO" if args.demo_feedback else "NONE"}', flush=True)
                    printed = now
            online = core.online(time.monotonic())
            if online != was_online:
                print('LINK OK' if online else 'LINK TIMEOUT', flush=True)
                was_online = online
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()


if __name__ == '__main__':
    main()
