import array
import hashlib
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

# Topics to monitor: topic name / message type / fields compared for change
# detection (header timestamps are deliberately excluded)
WATCH_LIST = [
    {'topic': '/dvl/data',           'type': 'dvl_msgs/msg/DVL',              'fields': ['velocity', 'altitude']},
    {'topic': '/dvl/position',       'type': 'dvl_msgs/msg/DVLDR',            'fields': ['position', 'roll', 'pitch', 'yaw']},
    {'topic': '/imu_px4/data',       'type': 'sensor_msgs/msg/Imu',           'fields': ['angular_velocity', 'linear_acceleration']},
    {'topic': '/imu/data',           'type': 'sensor_msgs/msg/Imu',           'fields': ['angular_velocity', 'linear_acceleration']},
    {'topic': '/imu/mag',            'type': 'sensor_msgs/msg/MagneticField', 'fields': ['magnetic_field']},
    {'topic': '/ekf/imu/data',       'type': 'sensor_msgs/msg/Imu',           'fields': ['orientation', 'angular_velocity', 'linear_acceleration']},
    {'topic': '/barometer/pressure', 'type': 'sensor_msgs/msg/FluidPressure', 'fields': ['fluid_pressure']},
]


def _comparable(value):
    """Convert a message field into a comparable value; digest large arrays."""
    if hasattr(value, 'get_fields_and_field_types'):
        return tuple(_comparable(getattr(value, name))
                     for name in value.get_fields_and_field_types())
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
    elif isinstance(value, array.array):
        raw = value.tobytes()
    elif hasattr(value, 'tobytes'):  # numpy array
        raw = value.tobytes()
    elif isinstance(value, (list, tuple)):
        return tuple(_comparable(v) for v in value)
    else:
        return value  # int / float / str / bool
    if len(raw) > 64:
        return hashlib.md5(raw).digest()
    return raw


class TopicWatcher:

    def __init__(self, topic, fields, baseline):
        self.topic = topic
        self.fields = fields
        self.last_msg_time = None
        self.last_change_time = baseline
        self.last_value = None
        self.stale = False

    def on_message(self, msg):
        value = tuple(_comparable(getattr(msg, f)) for f in self.fields)
        now = time.monotonic()
        self.last_msg_time = now
        if value != self.last_value:
            self.last_value = value
            self.last_change_time = now


class SensorHealthMonitor(Node):

    def __init__(self):
        super().__init__('sensor_health_monitor')
        self.declare_parameter('stale_threshold', 2.0)  # seconds without change before warning
        self.declare_parameter('check_period', 0.5)     # seconds between checks
        self.declare_parameter('startup_grace', 3.0)    # seconds before checks begin
        self.stale_threshold = self.get_parameter('stale_threshold').value
        startup_grace = self.get_parameter('startup_grace').value
        # No checks (and no stale counting for silent topics) until grace ends
        self.monitor_start = time.monotonic() + startup_grace

        self.watchers = []
        for item in WATCH_LIST:
            watcher = TopicWatcher(item['topic'], item['fields'], self.monitor_start)
            msg_class = get_message(item['type'])
            self.create_subscription(msg_class, item['topic'],
                                     watcher.on_message,
                                     qos_profile_sensor_data)
            self.watchers.append(watcher)

        self.create_timer(self.get_parameter('check_period').value, self.check)
        self.get_logger().info(
            f'Monitoring {len(self.watchers)} topics, warning after '
            f'{self.stale_threshold:.1f} s without data change '
            f'(checks start in {startup_grace:.1f} s)')

    def check(self):
        now = time.monotonic()
        if now < self.monitor_start:
            return
        for w in self.watchers:
            stale = now - w.last_change_time > self.stale_threshold
            if stale:
                if w.last_msg_time is None:
                    detail = 'NO DATA received yet'
                elif now - w.last_msg_time > self.stale_threshold:
                    detail = f'NO NEW MESSAGE for {now - w.last_msg_time:.1f} s'
                else:
                    detail = ('still publishing, but VALUE FROZEN for '
                              f'{now - w.last_change_time:.1f} s')
                self.get_logger().warn(f'!!! SENSOR STALE !!! [{w.topic}] {detail}')
            elif w.stale:
                self.get_logger().info(f'[{w.topic}] data is updating again')
            w.stale = stale


def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
