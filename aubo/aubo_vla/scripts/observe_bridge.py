#!/usr/bin/python3
"""Latest-frame ROS bridge. Publishes JSON diagnostics only, never commands."""
import base64
import json
import math
import threading
import time
import uuid
import urllib.request
import urllib.parse

import cv2
import rospy
from cv_bridge import CvBridge
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Bridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.epoch = 0
        self.clock = None
        self.clock_wall = time.monotonic()
        self.convert = CvBridge()
        self.timeout = float(rospy.get_param('~timeout', 30.0))
        self.max_age = float(rospy.get_param('~max_age', 30.0))
        self.pause_timeout = float(rospy.get_param('~pause_timeout', 2.0))
        self.interval = float(rospy.get_param('~interval', 1.0))
        self.url = rospy.get_param('~url', 'http://127.0.0.1:8008/predict')
        if urllib.parse.urlparse(self.url).hostname not in ('127.0.0.1', 'localhost'):
            raise ValueError('Only loopback inference endpoints are allowed')
        if min(self.timeout, self.max_age, self.pause_timeout, self.interval) <= 0:
            raise ValueError('Timeouts and interval must be positive')
        self.instruction = rospy.get_param('~instruction', 'pick up the red block')
        self.pub = rospy.Publisher('~diagnostics', String, queue_size=1)
        # Disable inherited HTTP proxies for the local service.
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic', '/workspace_camera/color/image_raw'),
                         Image, self.on_image, queue_size=1, buff_size=4*1024*1024)

    def on_clock(self, msg):
        stamp, now = msg.clock.to_sec(), time.monotonic()
        with self.lock:
            if self.clock is not None and (stamp < self.clock or now-self.clock_wall > self.pause_timeout):
                self.epoch += 1
                self.latest = None
            if stamp != self.clock:
                self.clock_wall = now
            self.clock = stamp

    def on_image(self, msg):
        with self.lock:
            self.latest = (msg, time.monotonic(), self.epoch)

    def emit(self, status, **kwargs):
        self.pub.publish(String(data=json.dumps(dict(status=status, observation_only=True, **kwargs),
                                               allow_nan=False)))

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                item, self.latest = self.latest, None
                paused = self.clock is None or time.monotonic()-self.clock_wall > self.pause_timeout
            if paused or item is None:
                time.sleep(0.1)
                continue
            msg, received, epoch = item
            request_id = str(uuid.uuid4())
            try:
                if time.monotonic()-received > self.max_age:
                    self.emit('discarded', reason='old input frame')
                    continue
                rgb = self.convert.imgmsg_to_cv2(msg, 'bgr8')
                ok, png = cv2.imencode('.png', rgb)
                if not ok:
                    raise ValueError('PNG encoding failed')
                payload = dict(request_id=request_id, stamp=msg.header.stamp.to_sec(),
                               frame_id=msg.header.frame_id, instruction=self.instruction,
                               image_png=base64.b64encode(png.tobytes()).decode('ascii'))
                request = urllib.request.Request(self.url, json.dumps(payload).encode(),
                                                 {'Content-Type': 'application/json'})
                start = time.monotonic()
                with self.http.open(request, timeout=self.timeout) as response:
                    result = json.loads(response.read(1024*1024))
                elapsed = time.monotonic()-start
                with self.lock:
                    stale = (epoch != self.epoch or time.monotonic()-self.clock_wall > self.pause_timeout
                             or time.monotonic()-received > self.max_age or elapsed > self.timeout)
                if stale:
                    self.emit('discarded', request_id=request_id, reason='expired, paused or reset')
                else:
                    action = result['action']
                    if (result['request_id'] != request_id or result['stamp'] != payload['stamp']
                            or result['frame_id'] != payload['frame_id']
                            or result.get('observation_only') is not True
                            or not isinstance(result.get('mock'), bool)
                            or len(action) != 7 or not all(math.isfinite(float(x)) for x in action)):
                        raise ValueError('Invalid or mismatched model response')
                    self.emit('mock' if result['mock'] else 'observed', result=result, http_ms=elapsed*1000)
            except Exception as exc:
                self.emit('error', request_id=request_id, error=str(exc))
                rospy.logwarn_throttle(5, 'OpenVLA observation failed: %s', str(exc))
            # Wall-clock pacing continues even when Gazebo is paused.
            time.sleep(self.interval)


if __name__ == '__main__':
    rospy.init_node('openvla_observe')
    Bridge().run()
