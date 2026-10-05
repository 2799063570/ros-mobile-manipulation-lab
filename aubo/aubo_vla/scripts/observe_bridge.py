#!/usr/bin/python3
"""Latest-frame ROS bridge. Publishes JSON diagnostics only, never commands."""
import base64
import json
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
from aubo_vla.timing import ClockGuard, positive, source_age
from aubo_vla.actions import vector


class Bridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.epoch = 0
        self.convert = CvBridge()
        self.timeout = positive(rospy.get_param('~timeout', 30.0), 'timeout')
        self.max_age = positive(rospy.get_param('~max_age', 30.0), 'max_age')
        self.pause_timeout = positive(rospy.get_param('~pause_timeout', 2.0), 'pause_timeout')
        self.interval = positive(rospy.get_param('~interval', 1.0), 'interval')
        self.clock_guard = ClockGuard(bool(rospy.get_param('/use_sim_time', False)), self.pause_timeout)
        self.url = rospy.get_param('~url', 'http://127.0.0.1:8008/predict')
        parsed = urllib.parse.urlparse(self.url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost'):
            raise ValueError('Only loopback inference endpoints are allowed')
        self.instruction = rospy.get_param('~instruction', 'pick up the red block')
        if not isinstance(self.instruction, str) or not 0 < len(self.instruction.strip()) <= 1000:
            raise ValueError('Invalid instruction')
        self.pub = rospy.Publisher('~diagnostics', String, queue_size=1)
        # Disable inherited HTTP proxies for the local service.
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        if self.clock_guard.simulated:
            rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1)
        rospy.Subscriber('~instruction', String, self.on_instruction, queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic', '/workspace_camera/color/image_raw'),
                         Image, self.on_image, queue_size=1, buff_size=4*1024*1024)

    def on_clock(self, msg):
        stamp, now = msg.clock.to_sec(), time.monotonic()
        with self.lock:
            if self.clock_guard.update(stamp, now):
                self.epoch += 1
                self.latest = None

    def on_instruction(self, msg):
        instruction = msg.data.strip()
        if not 0 < len(instruction) <= 1000:
            rospy.logwarn('Ignored empty or oversized VLA instruction')
            return
        with self.lock:
            if instruction != self.instruction:
                self.instruction = instruction
                self.epoch += 1
                self.latest = None

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
                paused = self.clock_guard.paused(time.monotonic())
                instruction = self.instruction
            if paused or item is None:
                time.sleep(0.1)
                continue
            msg, received, epoch = item
            request_id = str(uuid.uuid4())
            try:
                if time.monotonic()-received > self.max_age:
                    self.emit('discarded', reason='old input frame')
                    continue
                source_age(msg.header.stamp.to_sec(), rospy.Time.now().to_sec(), self.max_age)
                rgb = self.convert.imgmsg_to_cv2(msg, 'bgr8')
                ok, png = cv2.imencode('.png', rgb)
                if not ok:
                    raise ValueError('PNG encoding failed')
                payload = dict(request_id=request_id, stamp=msg.header.stamp.to_sec(),
                               frame_id=msg.header.frame_id, instruction=instruction,
                               image_png=base64.b64encode(png.tobytes()).decode('ascii'))
                request = urllib.request.Request(self.url, json.dumps(payload).encode(),
                                                 {'Content-Type': 'application/json'})
                start = time.monotonic()
                with self.http.open(request, timeout=self.timeout) as response:
                    result = json.loads(response.read(1024*1024))
                elapsed = time.monotonic()-start
                with self.lock:
                    stale = (epoch != self.epoch or self.clock_guard.paused(time.monotonic())
                             or time.monotonic()-received > self.max_age or elapsed > self.timeout)
                    if not stale:
                        try:
                            source_age(payload['stamp'], rospy.Time.now().to_sec(), self.max_age)
                        except ValueError:
                            stale = True
                    if stale:
                        self.emit('discarded', request_id=request_id, reason='expired, paused or reset')
                    else:
                        vector(result['action'], 7, 'model action')
                        if (result['request_id'] != request_id or result['stamp'] != payload['stamp']
                                or result['frame_id'] != payload['frame_id']
                                or result.get('observation_only') is not True
                                or not isinstance(result.get('mock'), bool)):
                            raise ValueError('Invalid or mismatched model response')
                        # Publish under the epoch lock so a concurrent instruction
                        # update/reset cannot interleave validation and acceptance.
                        self.emit('mock' if result['mock'] else 'observed', result=result,
                                  instruction=instruction, http_ms=elapsed*1000,
                                  frame_age_ms=(time.monotonic()-received)*1000,
                                  source_age_ms=(rospy.Time.now().to_sec()-payload['stamp'])*1000)
            except Exception as exc:
                self.emit('error', request_id=request_id, error=str(exc))
                rospy.logwarn_throttle(5, 'OpenVLA observation failed: %s', str(exc))
            # Wall-clock pacing continues even when Gazebo is paused.
            time.sleep(self.interval)


if __name__ == '__main__':
    rospy.init_node('openvla_observe')
    Bridge().run()
