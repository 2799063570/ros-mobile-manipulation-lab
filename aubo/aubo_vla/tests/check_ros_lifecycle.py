#!/usr/bin/python3
"""Run only against the disposable Gazebo instance started for deployment validation."""
import json
import threading
import time
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from observe_bridge import Bridge

rospy.init_node('openvla_lifecycle_check', anonymous=True)
bridge = Bridge.__new__(Bridge)
bridge.lock = threading.Lock()
bridge.latest = None
bridge.epoch = 0
bridge.clock = None
bridge.clock_wall = time.monotonic()
bridge.pause_timeout = 2.0
# Deterministic reset and old-frame invalidation using synthetic clock messages.
bridge.on_clock(Clock(clock=rospy.Time.from_sec(100)))
msg = Image()
bridge.on_image(msg)
before = bridge.epoch
bridge.on_clock(Clock(clock=rospy.Time.from_sec(50)))
assert bridge.epoch > before and bridge.latest is None
print('Clock rollback clears pending frame: PASS', flush=True)
# Live pause/resume: wait with monotonic time, not simulated time.
messages = []
def cb(msg):
    messages.append((time.monotonic(), json.loads(msg.data)))
sub = rospy.Subscriber('/openvla_observe/diagnostics', String, cb)
time.sleep(2)
assert messages, 'No live bridge diagnostics'
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
try:
    pause()
    time.sleep(3)
    count = len(messages)
    time.sleep(2)
    assert len(messages) == count, 'Bridge published during sustained pause'
    print('Gazebo pause stops requests: PASS', flush=True)
finally:
    unpause()
time.sleep(3)
assert len(messages) > count, 'Bridge did not resume'
print('Gazebo unpause resumes observations: PASS', flush=True)
