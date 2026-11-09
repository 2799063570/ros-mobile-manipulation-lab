"""ROS IO runs off the Qt thread; constructing this bridge never commands motion."""
import json
import threading
import time

import rospy
import rosservice
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from dynamic_reconfigure.client import Client
from moveit_msgs.srv import QueryPlannerInterfaces
from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtGui import QImage
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .model import ACTIONS, parameter_changes, planner_options, queue_rows, completion_counts


class RosBridge(QObject):
    event = Signal(str, object)

    def __init__(self, task_node, sorting_ns, image_topic, hardware_topic,
                 planner_service='/query_planner_interface'):
        super(RosBridge, self).__init__()
        self.task_node = task_node.rstrip('/')
        self.sorting_ns = sorting_ns.rstrip('/')
        self.planner_service = planner_service
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._data = {'state': '', 'summary': '', 'failure': '', 'rows': [],
                      'services': set(), 'hardware': {}, 'config': {}, 'image': None,
                      'image_at': 0., 'hardware_at': 0., 'progress': {}, 'frame': ''}
        self._client = None
        self._client_uri = None
        self._jobs = set()
        self._converter = CvBridge()
        self._image_converted_at = 0.
        self._subscribers = []
        self._state_subscriber = self._subscribe('state', String, self._state)
        self._subscribe('detection_summary', String, lambda m: self._put(summary=m.data))
        self._subscribe('failure', String, self._failure)
        self._subscribe('target_cache', String, self._queue)
        self._subscribe('progress', String, self._progress)
        self._subscribe('completed_counts', String, self._completed_counts)
        self._subscribers.append(rospy.Subscriber(image_topic, Image, self._image, queue_size=1))
        self._subscribers.append(rospy.Subscriber(hardware_topic, DiagnosticArray,
                                                  self._hardware, queue_size=1))
        self._thread(self._discover)

    def _subscribe(self, suffix, message_type, callback):
        sub = rospy.Subscriber(self.sorting_ns + '/' + suffix, message_type, callback, queue_size=1)
        self._subscribers.append(sub)
        return sub

    def _put(self, **values):
        with self._lock:
            self._data.update(values)

    def _emit(self, kind, value):
        if not self._closed.is_set():
            try:
                self.event.emit(kind, value)
            except RuntimeError:
                pass  # A daemon worker may finish after rqt unloaded the plugin.

    def _thread(self, operation):
        threading.Thread(target=operation, daemon=True).start()

    def snapshot(self):
        with self._lock:
            data = dict(self._data)
            data['pending'] = bool(self._jobs - {'stop'})
        try:
            data['connected'] = not self._closed.is_set() and self._state_subscriber.get_num_connections() > 0
        except AttributeError:  # rospy may have unregistered subscribers during shutdown.
            data['connected'] = False
        return data

    def _state(self, message):
        with self._lock:
            changed = message.data != self._data['state']
            self._data['state'] = message.data
        if changed:
            self._emit('state', message.data)

    def _failure(self, message):
        with self._lock:
            changed = message.data != self._data['failure']
            self._data['failure'] = message.data
        if message.data and changed:
            self._emit('error', message.data)

    def _queue(self, message):
        try:
            document=json.loads(message.data)
            frame, rows = queue_rows(document)
            self._put(frame=frame, rows=rows, queue_error='', vision_phase=document.get('vision_phase',''),
                      target_occluded=document.get('target_occluded',False),
                      merged_detections_total=document.get('merged_detections_total',0))
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self._put(rows=[], queue_error=str(error))

    def _completed_counts(self, message):
        try:
            self._put(completed_counts=completion_counts(message.data), completion_error='')
        except (ValueError, TypeError, OverflowError):
            self._put(completion_error='完成统计数据无效')

    def _progress(self, message):
        try:
            progress = json.loads(message.data)
            if not isinstance(progress, dict):
                return
            self._put(progress=progress)
            self._emit('progress', progress)
        except (ValueError, TypeError):
            pass

    def _image(self, message):
        now = time.monotonic()
        if now - self._image_converted_at < .2:
            return
        self._image_converted_at = now
        try:
            rgb = self._converter.imgmsg_to_cv2(message, 'rgb8').copy()
            image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
            self._put(image=image, image_at=now)
        except Exception as error:
            self._put(image_error=str(error))

    def _hardware(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if 'feedback_source' in values:
                self._put(hardware=values, hardware_at=time.monotonic())

    def _configuration(self, config):
        self._put(config=dict(config))
        self._emit('configuration', dict(config))

    def _discover(self):
        while not self._closed.is_set() and not rospy.is_shutdown():
            try:
                self._put(services=set(rosservice.get_service_list()))
                uri = rosservice.get_service_uri(self.task_node + '/set_parameters')
                if uri != self._client_uri:
                    client, self._client = self._client, None
                    if client is not None:
                        client.close()
                    self._put(config={})
                    self._client_uri = uri
                # The server publishes parameter_updates on connect. No update RPC here.
                if self._client is None and uri is not None:
                    client = Client(self.task_node, timeout=1., config_callback=self._configuration)
                    if self._closed.is_set():
                        client.close()
                        return
                    self._client = client
                self._put(cell={key: rospy.get_param(self.task_node + '/' + key, None)
                                for key in ('sort_classes', 'table_center', 'table_size',
                                            'continuous_sorting', 'target_frame')})
                self._discover_planners()
            except Exception:
                self._put(services=set())
            self._closed.wait(2.)

    def _discover_planners(self):
        try:
            external = rospy.get_param(self.task_node + '/planning_service', '')
            self._put(external_planning_service=external)
            rospy.wait_for_service(self.planner_service, timeout=.5)
            response = rospy.ServiceProxy(self.planner_service, QueryPlannerInterfaces)()
            group = rospy.get_param(self.task_node + '/planning_group', 'aubo_i5')
            options = planner_options(response.planner_interfaces, group)
            default = rospy.get_param('/move_group/default_planning_pipeline', '')
            if not default and len(options) == 1:
                default = next(iter(options))
            self._put(planners=options, default_pipeline=default, planner_error='')
        except Exception as error:
            self._put(planners={}, planner_error='无法读取 MoveIt 算法列表：{}'.format(error))

    def command(self, name):
        if name not in {key for key, _ in ACTIONS}:
            return
        with self._lock:
            if name in self._jobs or self._closed.is_set():
                return
            self._jobs.add(name)

        def call():
            try:
                service = self.sorting_ns + '/' + name
                rospy.wait_for_service(service, timeout=1.)
                result = rospy.ServiceProxy(service, Trigger)()
                self._emit('command', {'name': name, 'success': result.success, 'message': result.message})
            except Exception as error:
                self._emit('error', '{}: {}'.format(name, error))
            finally:
                with self._lock:
                    self._jobs.discard(name)
        self._thread(call)

    def apply(self, values):
        with self._lock:
            if self._jobs or self._closed.is_set():
                self._emit('error', '有请求尚未结束，请稍后再应用参数')
                return
            self._jobs.add('parameters')

        def call():
            try:
                if self._client is None:
                    raise RuntimeError('在线参数服务器尚未连接')
                requested = dict(values)
                accepted = self._client.update_configuration(dict(values))
                self._put(config=dict(accepted))
                rejected = parameter_changes(requested, accepted)
                self._emit('applied', {'accepted': accepted, 'rejected': rejected})
            except Exception as error:
                client = self._client
                self._client = None
                if client is not None:
                    client.close()
                self._emit('error', '在线调参失败: {}'.format(error))
            finally:
                with self._lock:
                    self._jobs.discard('parameters')
        self._thread(call)

    def refresh(self):
        def call():
            try:
                if self._client is None:
                    raise RuntimeError('在线参数服务器尚未连接')
                # Read-only: dynamic_reconfigure Client caches parameter_updates.
                config = self._client.get_configuration(timeout=1.)
                if config is None:
                    raise RuntimeError('读取参数超时')
                self._emit('refresh', config)
            except Exception as error:
                self._emit('error', str(error))
        self._thread(call)

    def close(self):
        if self._closed.is_set():
            return
        self._closed.set()
        for subscriber in self._subscribers:
            subscriber.unregister()
        if self._client is not None:
            self._client.close()
