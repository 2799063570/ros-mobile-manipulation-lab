"""Small ROS message/IO doubles for node-boundary tests without ROS installed."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch


class Stamp:
    now_value = 10.2

    def __init__(self, value=0.):
        self.value = value

    def to_sec(self):
        return self.value

    @classmethod
    def now(cls):
        return cls(cls.now_value)

    @classmethod
    def from_sec(cls, value):
        return cls(value)


class Marker:
    ADD, DELETE, LINE_LIST, ARROW = 0, 2, 5, 0

    def __init__(self):
        self.header = NS(frame_id='', stamp=Stamp())
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.))
        self.scale = NS(x=0., y=0., z=0.)
        self.color = NS(r=0., g=0., b=0., a=0.)


def point(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def transform():
    return NS(transform=NS(translation=point(0.5, 0., 0.3), rotation=NS(x=0., y=0., z=0., w=1.)))


def load_node(name, params, png=b''):
    rospy = NS(get_param=lambda key, default=None: params.get(key, default),
               Publisher=Mock(side_effect=lambda *a, **kw: Mock()), Subscriber=Mock(), Service=Mock(),
               logwarn=Mock(), logwarn_throttle=Mock(), on_shutdown=Mock(), get_name=lambda: '/test',
               Time=Stamp, Duration=lambda seconds: seconds, is_shutdown=Mock(return_value=False))
    tf = Mock()
    tf.lookup_transform.return_value = transform()
    tf_module = NS(Buffer=lambda **kw: tf, TransformListener=Mock(), LookupException=LookupError,
                   ConnectivityException=ConnectionError, ExtrapolationException=TimeoutError)
    modules = {'rospy': rospy, 'tf2_ros': tf_module,
               'cv2': NS(imencode=lambda *a: (True, NS(tobytes=lambda: png))),
               'cv_bridge': NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda *a: None)),
               'message_filters': NS(Subscriber=Mock(), ApproximateTimeSynchronizer=Mock()),
               'rosgraph_msgs.msg': NS(Clock=object),
               'sensor_msgs.msg': NS(Image=object, JointState=object, CameraInfo=object),
               'std_msgs.msg': NS(String=lambda **kw: NS(**kw)),
               'std_srvs.srv': NS(Trigger=object, TriggerResponse=lambda success, message: NS(
                   success=success, message=message)),
               'geometry_msgs.msg': NS(Point=point),
               'visualization_msgs.msg': NS(Marker=Marker, MarkerArray=lambda **kw: NS(**kw))}
    path = Path(__file__).resolve().parents[1]/('scripts/'+name+'.py')
    spec = importlib.util.spec_from_file_location('tested_'+name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module, rospy, tf
