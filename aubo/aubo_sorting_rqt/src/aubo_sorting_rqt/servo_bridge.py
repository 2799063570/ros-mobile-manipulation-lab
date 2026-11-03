"""Read-only subscriptions plus explicit servo requests on independent workers."""
import threading
import time

import rospy
import rosservice
import tf2_ros
from cv_bridge import CvBridge
from dynamic_reconfigure.client import Client
from python_qt_binding.QtCore import QObject, Signal
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool, Trigger
from aubo_ros_control.msg import VisualServoStatus

from .bridge import RosBridge
from .model import parameter_changes
from .servo_model import POSE_KEYS, action_gates, sight_clearance, vector


class ServoBridge(QObject):
    event = Signal(str, object)

    def __init__(self, controller_node, servo_ns, image_topic, geometry_node):
        super(ServoBridge, self).__init__()
        self.controller = controller_node.rstrip('/')
        self.namespace = servo_ns.rstrip('/')
        self.geometry_node = geometry_node.rstrip('/')
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._jobs = set()
        self._stop_epoch = 0
        self._client = None
        self._client_uri = None
        self._data = dict(state='', status={}, status_at=0., config={}, services=set(),
                          image=None, image_at=0., visibility={}, selected_class='')
        self._converter = CvBridge()
        self._image_converted_at = 0.
        self._tf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf)
        self._selection = rospy.Publisher(self.namespace+'/target_selection', String, queue_size=1)
        self._subscribers = [
            rospy.Subscriber(self.namespace+'/state', String, self._state, queue_size=1),
            rospy.Subscriber(self.namespace+'/status', VisualServoStatus, self._status, queue_size=1),
            rospy.Subscriber(self.namespace+'/perception_state', String, self._perception, queue_size=1),
            rospy.Subscriber(self.namespace+'/target_label', String, self._target_label, queue_size=1),
            rospy.Subscriber(self.namespace+'/target_confidence', Float32,
                             lambda message: self._put(confidence=message.data), queue_size=1),
            rospy.Subscriber(image_topic, Image, self._image, queue_size=1),
        ]
        self._thread(self._discover)
        self._thread(self._geometry)

    def _thread(self, operation):
        threading.Thread(target=operation, daemon=True).start()

    def _put(self, **values):
        with self._lock:
            self._data.update(values)

    def _emit(self, kind, value):
        if not self._closed.is_set():
            try:
                self.event.emit(kind, value)
            except RuntimeError:
                pass

    def snapshot(self):
        with self._lock:
            result = dict(self._data)
            result['pending'] = bool(self._jobs-{'stop'})
        try:
            result['connected'] = self._subscribers[0].get_num_connections() > 0
        except AttributeError:
            result['connected'] = False
        return result

    def _state(self, message):
        with self._lock:
            changed = self._data['state'] != message.data
            self._data['state'] = message.data
        if changed:
            self._emit('log', '控制器状态：'+message.data)

    def _status(self, message):
        age = (rospy.Time.now()-message.header.stamp).to_sec()
        if message.header.stamp.is_zero() or age < -.05:
            return
        with self._lock:
            if (self._data.get('selection_pending') and message.target_valid and
                    message.header.stamp.to_sec()-message.target_age > self._data.get('selection_epoch', float('inf')) and
                    self._data.get('detected_class') == self._data.get('requested_class') and
                    self._data.get('selected_class') == self._data.get('requested_class')):
                self._data['selection_pending'] = False
        point = lambda value: (value.x, value.y, value.z)
        self._put(status=dict(feedback_valid=message.feedback_valid, target_valid=message.target_valid,
                              error_valid=message.error_valid, feedback_age=message.feedback_age,
                              target_age=message.target_age, error=point(message.position_error),
                              tcp=point(message.tcp_position), target=point(message.observed_position),
                              goal=point(message.goal_position), detail=message.detail,
                              frame=message.header.frame_id, mode=message.servo_mode),
                  status_at=time.monotonic()-max(0., age))

    def _perception(self, message):
        fields = message.data.split('|', 2)
        self._put(perception=message.data, selected_class=fields[1] if len(fields) > 1 else '')

    def _target_label(self, message):
        self._put(detected_class=message.data)

    def _image(self, message):
        RosBridge._image(self, message)  # Shared bounded 5 Hz image conversion.

    def _configuration(self, config):
        self._put(config=dict(config))
        self._emit('configuration', dict(config))

    def _discover(self):
        while not self._closed.is_set() and not rospy.is_shutdown():
            try:
                services = set(rosservice.get_service_list())
                self._put(services={key for key in ('set_enabled', 'reset')
                                    if self.namespace+'/'+key in services},
                          perception_controls=self.namespace+'/perception/set_enabled' in services)
                uri = rosservice.get_service_uri(self.controller+'/set_parameters')
                if uri != self._client_uri:
                    client, self._client = self._client, None
                    if client is not None:
                        client.close()
                    self._client_uri = uri
                    self._put(config={})
                if uri and self._client is None:
                    client = Client(self.controller, timeout=1., config_callback=self._configuration)
                    if self._closed.is_set():
                        client.close()
                        return
                    self._client = client
                self._put(cell={key: rospy.get_param(self.controller+'/'+key, default) for key, default in (
                    ('base_link', 'base_link'), ('camera_link', 'camera_color_optical_frame'),
                    ('control_link', 'tcp_link'), ('servo_mode', ''), ('hybrid_enabled', False),
                    ('feedback_timeout', .15), ('initial_search_enabled', False),
                    ('initial_search_posture', []), ('recovery_posture', []))})
                if not self._data.get('selected_class'):
                    self._put(selected_class=rospy.get_param(self.geometry_node+'/selected_class', ''))
            except Exception as error:
                self._put(discovery_error=str(error), services=set())
            self._closed.wait(1.)

    def _geometry(self):
        links = ('shoulder_Link', 'upperArm_Link', 'foreArm_Link', 'wrist1_Link',
                 'wrist2_Link', 'wrist3_Link', 'gripper_base_link', 'tcp_link')
        while not self._closed.is_set() and not rospy.is_shutdown():
            data = self.snapshot()
            cell, status = data.get('cell', {}), data.get('status', {})
            try:
                if time.monotonic()-data.get('status_at', 0.) > .75 or not status.get('error_valid'):
                    raise ValueError('等待新鲜目标、反馈及 TF')
                if cell.get('servo_mode') != 'eye_to_hand':
                    raise ValueError('本姿态助手适用于眼在手外')
                base = cell.get('base_link', 'base_link')
                # Query one source time for all dynamic link transforms.
                stamp = rospy.Time.now()-rospy.Duration(.08)
                def lookup(link, when):
                    t = self._tf.lookup_transform(base, link, when, rospy.Duration(0)).transform
                    return vector((t.translation.x, t.translation.y, t.translation.z)), t.rotation
                camera, _ = lookup(cell.get('camera_link', 'camera_color_optical_frame'), stamp)
                points = [lookup(link, stamp)[0] for link in links]
                _, rotation = lookup(cell.get('control_link', 'tcp_link'), stamp)
                target = vector(status['target'])
                self._put(visibility=dict(camera=camera, target=target, links=points,
                                           rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
                                           clearance=sight_clearance(camera, target, points),
                                           at=time.monotonic(), detail='60 mm 连杆包络估计；不是可见性或碰撞保证'))
            except Exception as error:
                self._put(visibility=dict(detail=str(error)))
            self._closed.wait(.2)

    def command(self, name):
        if name not in ('start', 'stop', 'reset', 'vision_on', 'vision_off'):
            return
        with self._lock:
            if self._closed.is_set() or name in self._jobs:
                return
            if name == 'stop':
                self._stop_epoch += 1
            elif name in ('start', 'reset') and not action_gates(self.snapshot(), time.monotonic())[name]:
                self._emit('log', '请求被界面拒绝：请检查连接、反馈、目标及禁用状态')
                return
            epoch = self._stop_epoch
            self._jobs.add(name)

        def call():
            try:
                if name == 'reset':
                    service, kind, args = self.namespace+'/reset', Trigger, ()
                elif name.startswith('vision_'):
                    service, kind, args = self.namespace+'/perception/set_enabled', SetBool, (name == 'vision_on',)
                else:
                    service, kind, args = self.namespace+'/set_enabled', SetBool, (name == 'start',)
                rospy.wait_for_service(service, timeout=1.)
                if name == 'start' and epoch != self._stop_epoch:
                    return
                response = rospy.ServiceProxy(service, kind)(*args)
                if name == 'start' and epoch != self._stop_epoch:
                    rospy.ServiceProxy(service, SetBool)(False)
                self._emit('log', '{}：{} · {}'.format(name, '请求已接受' if response.success else '请求被拒绝', response.message))
            except Exception as error:
                self._emit('log', '{} 失败：{}'.format(name, error))
            finally:
                with self._lock:
                    self._jobs.discard(name)
        self._thread(call)

    def select_class(self, name):
        name = str(name).strip().lower()
        if not name or len(name) > 64 or any(c in name for c in ('|', '\n', '\r')):
            self._emit('log', '目标类别无效')
            return
        data = self.snapshot()
        if not data.get('perception_controls') or not action_gates(data, time.monotonic())['pose']:
            self._emit('log', '切换类别需要禁用伺服并连接新版 YOLO 感知接口')
            return
        self._put(selection_pending=True, requested_class=name, selection_epoch=rospy.Time.now().to_sec())
        self._selection.publish(String(data=name))
        self._emit('log', '请求切换目标：'+name+'；等待新类别、新源图像和控制遥测确认')

    def apply(self, values):
        with self._lock:
            data = self.snapshot()
            gates = action_gates(data, time.monotonic())
            changed = parameter_changes(values, data.get('config', {}))
            if not changed:
                self._emit('log', '没有待应用的参数编辑')
                return
            if not gates['tune'] or (POSE_KEYS.intersection(changed) and not gates['pose']):
                self._emit('log', '参数未发送：姿态/偏移需先禁用并停稳，其余参数需有效连接')
                return
            self._jobs.add('parameters')
        def call():
            try:
                if self._client is None:
                    raise RuntimeError('参数服务器未连接')
                accepted = self._client.update_configuration({k: values[k] for k in changed})
                self._put(config=dict(accepted))
                self._emit('applied', dict(accepted=accepted, rejected=parameter_changes(values, accepted)))
            except Exception as error:
                self._emit('log', '调参失败：'+str(error))
            finally:
                with self._lock:
                    self._jobs.discard('parameters')
        self._thread(call)

    def refresh(self):
        def call():
            try:
                config = self._client.get_configuration(timeout=1.) if self._client else None
                if config is None:
                    raise RuntimeError('参数服务器未连接')
                self._emit('refresh', dict(config))
            except Exception as error:
                self._emit('log', '读取失败：'+str(error))
        self._thread(call)

    def close(self):
        self._closed.set()
        for sub in self._subscribers:
            sub.unregister()
        self._selection.unregister()
        self._listener.unregister()
        if self._client is not None:
            self._client.close()
