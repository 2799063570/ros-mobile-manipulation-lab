#!/usr/bin/python3
"""A paused/resumed Gazebo must invalidate an already running request."""
import argparse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from model_service import Predictor, handler_for
from observe_bridge import Bridge

rospy.init_node('openvla_inflight_test', anonymous=True)
rospy.set_param('~url', 'http://127.0.0.1:8009/predict')
started, release = threading.Event(), threading.Event()
base = Predictor(argparse.Namespace(mock=True, revision='test', unnorm_key='bridge_orig'))
class Slow:
    metadata = base.metadata
    def predict(self, payload):
        started.set()
        if not release.wait(15):
            raise TimeoutError('Test failed to release mock')
        return base.predict(payload)
server = ThreadingHTTPServer(('127.0.0.1', 8009), handler_for(Slow()))
threading.Thread(target=server.serve_forever, daemon=True).start()
messages = []
sub = rospy.Subscriber(rospy.get_name()+'/diagnostics', String, lambda m: messages.append(json.loads(m.data)))
bridge = Bridge()
worker = threading.Thread(target=bridge.run, daemon=True)
worker.start()
assert started.wait(15), 'No image reached mock server'
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
try:
    pause()
    time.sleep(3)
finally:
    unpause()
time.sleep(0.5)
release.set()
deadline = time.monotonic()+5
while not messages and time.monotonic() < deadline:
    time.sleep(0.05)
assert messages and messages[0]['status']=='discarded', messages
print('In-flight response across pause/resume discarded: PASS', flush=True)
rospy.signal_shutdown('test complete')
server.shutdown()
server.server_close()
worker.join(timeout=3)
