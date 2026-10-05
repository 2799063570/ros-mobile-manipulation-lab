#!/usr/bin/python3
"""Publish expiring RViz markers and JSON candidates, never motion commands."""
import json
import threading
import time

import rospy
import tf2_ros
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from aubo_vla.actions import SCHEMA, candidate, validate_model_contract
from aubo_vla.timing import ClockGuard, positive, source_age


class Shadow:
    def __init__(self):
        self.lock = threading.Lock()
        self.guard = ClockGuard(bool(rospy.get_param('/use_sim_time', False)))
        self.base = rospy.get_param('~base_frame', 'base_link')
        self.tcp = rospy.get_param('~tcp_frame', 'tcp_link')
        self.revision = rospy.get_param('~expected_revision', '')
        self.key = rospy.get_param('~expected_unnorm_key', '')
        self.dt = positive(rospy.get_param('~expected_action_dt', 0.5), 'expected_action_dt')
        self.max_age = positive(rospy.get_param('~max_age', 1.0), 'max_age')
        self.translation = positive(rospy.get_param('~max_translation', 0.01), 'max_translation')
        self.rotation = positive(rospy.get_param('~max_rotation', 0.05), 'max_rotation')
        self.low = rospy.get_param('~workspace_min')
        self.high = rospy.get_param('~workspace_max')
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.pub = rospy.Publisher('~candidates', String, queue_size=1)
        self.markers = rospy.Publisher('~markers', MarkerArray, queue_size=1)
        self.visible = None
        self.last_request = None
        rospy.Subscriber(rospy.get_param('~diagnostics_topic', '/openvla_observe/diagnostics'),
                         String, self.on_result, queue_size=1)
        if self.guard.simulated:
            rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1)

    def emit(self, status, **extra):
        self.pub.publish(String(data=json.dumps(dict(
            status=status, observation_only=True, executable=False,
            collision_checked=False, ik_checked=False, **extra), allow_nan=False)))

    def clear(self):
        markers = []
        for ident in (0, 1):
            marker = Marker()
            marker.ns, marker.id, marker.action = 'aubo_vla_shadow', ident, Marker.DELETE
            markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))
        self.visible = None

    def on_clock(self, msg):
        with self.lock:
            if self.guard.update(msg.clock.to_sec(), time.monotonic()):
                self.clear()

    def on_result(self, msg):
        with self.lock:
            request_id = None
            try:
                envelope = json.loads(msg.data)
                if envelope.get('status') != 'observed':
                    raise ValueError('Observation is not a real accepted result')
                result = envelope['result']
                request_id = result['request_id']
                if not isinstance(request_id, str) or not request_id or request_id == self.last_request:
                    raise ValueError('Missing or duplicate request ID')
                validate_model_contract(result, self.revision, self.key, self.dt)
                wall = time.monotonic()
                if self.guard.paused(wall):
                    raise ValueError('Simulation paused or clock unavailable')
                stamp = result['stamp']
                age = source_age(stamp, rospy.Time.now().to_sec(), self.max_age)
                if not result.get('frame_id'):
                    raise ValueError('Missing image frame')
                when = rospy.Time.from_sec(stamp)
                # Require both image and TCP frames at the observation time.
                self.tf.lookup_transform(self.base, result['frame_id'], when, rospy.Duration(0))
                transform = self.tf.lookup_transform(self.base, self.tcp, when, rospy.Duration(0))
                t, q = transform.transform.translation, transform.transform.rotation
                pose = dict(position=[t.x, t.y, t.z], orientation=[q.x, q.y, q.z, q.w])
                target = candidate(pose, result['action'], self.low, self.high,
                                   self.translation, self.rotation)
                age = source_age(stamp, rospy.Time.now().to_sec(), self.max_age)
                ttl = self.max_age-max(0., age)
                if ttl <= 0:
                    raise ValueError('Observation expired before preview')
                line = Marker()
                line.header.frame_id, line.header.stamp = self.base, when
                line.ns, line.id, line.type, line.action = 'aubo_vla_shadow', 0, Marker.LINE_LIST, Marker.ADD
                line.pose.orientation.w = 1.0
                line.points = [Point(*pose['position']), Point(*target['position'])]
                line.scale.x = 0.004
                line.color.r, line.color.g, line.color.a = 1.0, 0.7, 1.0
                line.lifetime = rospy.Duration(ttl)
                arrow = Marker()
                arrow.header = line.header
                arrow.ns, arrow.id, arrow.type, arrow.action = line.ns, 1, Marker.ARROW, Marker.ADD
                arrow.pose.position = Point(*target['position'])
                orientation = arrow.pose.orientation
                orientation.x, orientation.y, orientation.z, orientation.w = target['orientation']
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.04, 0.008, 0.008
                arrow.color.r, arrow.color.g, arrow.color.a = 1.0, 0.7, 1.0
                arrow.lifetime = line.lifetime
                self.markers.publish(MarkerArray(markers=[line, arrow]))
                self.visible = (stamp, time.monotonic()+ttl)
                self.last_request = request_id
                self.emit('preview', request_id=request_id, stamp=stamp, frame_id=self.base,
                          action_schema=SCHEMA, action_dt=self.dt, anchor_pose=pose, candidate=target,
                          raw_action=result['action'], expires_after=ttl)
            except Exception as exc:
                self.clear()
                self.emit('rejected', request_id=request_id, reason=str(exc))

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.visible:
                    stamp, deadline = self.visible
                    if (self.guard.paused(time.monotonic()) or time.monotonic() >= deadline or
                            not -0.05 <= rospy.Time.now().to_sec()-stamp <= self.max_age):
                        self.clear()
                        self.emit('expired')
            time.sleep(0.05)


if __name__ == '__main__':
    rospy.init_node('vla_shadow')
    Shadow().run()
