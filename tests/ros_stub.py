# Minimal stand-ins for rclpy and sensor_msgs, enough to construct and drive zed_usb_publisher.ZedUsb
# without ROS. Publishers record what they were given; loggers record every message by level.
import types
from types import SimpleNamespace


class Logger:
    def __init__(self):
        self.messages = []

    def _log(self, level, msg, **_):
        self.messages.append((level, msg))

    def info(self, msg, **kw):
        self._log('info', msg, **kw)

    def warn(self, msg, **kw):
        self._log('warn', msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        self._log('error', msg, **kw)

    def text(self, level=None):
        return '\n'.join(m for lv, m in self.messages if level in (None, lv))


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0), nanoseconds=0)


class Node:
    parameter_overrides = {}

    def __init__(self, name):
        self.name = name
        self._params = {}
        self._logger = Logger()
        self.publishers_by_topic = {}

    def declare_parameter(self, name, default):
        self._params[name] = self.parameter_overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock()

    def create_publisher(self, _type, topic, _qos):
        pub = Publisher(topic)
        self.publishers_by_topic[topic] = pub
        return pub

    def destroy_node(self):
        pass


class Image(SimpleNamespace):
    def __init__(self):
        super().__init__(header=SimpleNamespace(stamp=None, frame_id=''))


class CameraInfo(Image):
    pass


def modules():
    """sys.modules entries for rclpy, rclpy.node and sensor_msgs.msg."""
    rclpy = types.ModuleType('rclpy')
    rclpy.ok = lambda: True
    rclpy.init = lambda *a, **k: None
    rclpy.spin_once = lambda *a, **k: None
    rclpy.try_shutdown = lambda *a, **k: None
    node = types.ModuleType('rclpy.node')
    node.Node = Node
    rclpy.node = node
    sensor_msgs = types.ModuleType('sensor_msgs')
    msg = types.ModuleType('sensor_msgs.msg')
    msg.Image, msg.CameraInfo = Image, CameraInfo
    sensor_msgs.msg = msg
    return {'rclpy': rclpy, 'rclpy.node': node, 'sensor_msgs': sensor_msgs, 'sensor_msgs.msg': msg}
