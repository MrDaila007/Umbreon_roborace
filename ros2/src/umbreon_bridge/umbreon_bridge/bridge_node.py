"""
bridge_node.py — ROS2 node bridging Umbreon car telemetry to ROS2 topics.

Publishes sensor_msgs/Range, sensor_msgs/Imu, nav_msgs/Odometry,
sensor_msgs/LaserScan, std_msgs/String. Subscribes to cmd_vel for teleop.
Exposes services for connect/disconnect/start/stop/save/load/reset/ping.
Car config parameters are mapped to ROS2 node parameters.
"""

import math
import queue

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Range, Imu, LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from umbreon_bridge.tcp_client import TcpClient
from umbreon_bridge.car_config import DEFAULTS, FLOAT_KEYS, READONLY_KEYS
from umbreon_bridge.protocol import (
    encode_ping, encode_get, encode_set, encode_save, encode_load,
    encode_reset, encode_start, encode_stop, encode_status, encode_drv,
)

# Sensor geometry (from car_config.py)
SENSOR_FWD = 0.253   # m ahead of rear axle
SENSOR_LAT = [0.09, 0.04, -0.04, -0.09]   # lateral offset (+ = left)
SENSOR_DEG = [45.0, 0.0, 0.0, -45.0]       # relative to heading
SENSOR_FRAMES = ['lidar_left', 'lidar_front_left', 'lidar_front_right', 'lidar_right']
LIDAR_MIN_RANGE = 0.04   # 4 cm
LIDAR_MAX_RANGE = 8.0    # 800 cm


def _yaw_quaternion(yaw: float) -> Quaternion:
    """Create a Quaternion from a yaw angle (radians)."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class UmbreonBridgeNode(Node):
    def __init__(self):
        super().__init__('umbreon_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('host', '192.168.4.1')
        self.declare_parameter('port', 23)
        self.declare_parameter('auto_connect', True)

        # Declare all car config keys as parameters
        for key, default in DEFAULTS.items():
            if key in READONLY_KEYS:
                continue
            if key in FLOAT_KEYS:
                self.declare_parameter(key, float(default))
            else:
                self.declare_parameter(key, int(default))

        self.add_on_set_parameters_callback(self._on_param_change)

        # ── TCP client ────────────────────────────────────────────────
        self._client = TcpClient()

        # ── Publishers ────────────────────────────────────────────────
        self._pub_ranges = []
        range_names = ['left', 'front_left', 'front_right', 'right']
        for name in range_names:
            self._pub_ranges.append(
                self.create_publisher(Range, f'/umbreon/range/{name}', 10))
        self._pub_imu = self.create_publisher(Imu, '/umbreon/imu', 10)
        self._pub_odom = self.create_publisher(Odometry, '/umbreon/odom', 10)
        self._pub_scan = self.create_publisher(LaserScan, '/umbreon/scan', 10)
        self._pub_status = self.create_publisher(String, '/umbreon/status', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Float32, '/umbreon/throttle', self._throttle_cb, 10)
        self.create_subscription(Float32, '/umbreon/steering', self._steering_cb, 10)

        # ── Services ─────────────────────────────────────────────────
        self.create_service(Trigger, '/umbreon/connect', self._srv_connect)
        self.create_service(Trigger, '/umbreon/disconnect', self._srv_disconnect)
        self.create_service(Trigger, '/umbreon/ping', self._srv_ping)
        self.create_service(Trigger, '/umbreon/start', self._srv_start)
        self.create_service(Trigger, '/umbreon/stop', self._srv_stop)
        self.create_service(Trigger, '/umbreon/save', self._srv_save)
        self.create_service(Trigger, '/umbreon/load', self._srv_load)
        self.create_service(Trigger, '/umbreon/reset', self._srv_reset)

        # ── TF ────────────────────────────────────────────────────────
        self._static_tf = StaticTransformBroadcaster(self)
        self._dynamic_tf = TransformBroadcaster(self)
        self._publish_static_tf()

        # ── Odometry state ────────────────────────────────────────────
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_heading = 0.0
        self._last_speed = 0.0
        self._last_yaw_rate = 0.0

        # ── Manual drive state (for mixed cmd_vel + throttle/steering) ─
        self._last_cmd_steer = 0
        self._last_cmd_speed = 0.0

        # ── Timer (50 Hz) ─────────────────────────────────────────────
        self.create_timer(0.02, self._timer_cb)

        # ── Auto-connect ──────────────────────────────────────────────
        if self.get_parameter('auto_connect').value:
            self._do_connect()

    # ══════════════════════════════════════════════════════════════════
    #  Connection helpers
    # ══════════════════════════════════════════════════════════════════

    def _do_connect(self) -> tuple:
        """Connect and fetch config. Returns (success, message)."""
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            self._client.connect(host, port)
            self.get_logger().info(f'Connected to {host}:{port}')
            # Request current config
            self._client.send_command(encode_get())
            return True, f'Connected to {host}:{port}'
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False, str(e)

    def _send_and_wait(self, cmd: str, timeout: float = 2.0) -> dict:
        """Send command and wait for a response from response_queue."""
        # Drain stale responses
        while not self._client.response_queue.empty():
            try:
                self._client.response_queue.get_nowait()
            except queue.Empty:
                break
        self._client.send_command(cmd)
        try:
            return self._client.response_queue.get(timeout=timeout)
        except queue.Empty:
            return {'type': None, 'reason': 'timeout'}

    # ══════════════════════════════════════════════════════════════════
    #  Parameter change callback → $SET
    # ══════════════════════════════════════════════════════════════════

    def _on_param_change(self, params: list) -> SetParametersResult:
        if not self._client.connected:
            return SetParametersResult(successful=True)

        set_pairs = {}
        for p in params:
            if p.name in DEFAULTS and p.name not in READONLY_KEYS:
                set_pairs[p.name] = p.value

        if set_pairs:
            self._client.send_command(encode_set(set_pairs))

        return SetParametersResult(successful=True)

    # ══════════════════════════════════════════════════════════════════
    #  Static TF (sensor frames relative to base_link)
    # ══════════════════════════════════════════════════════════════════

    def _publish_static_tf(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        for i, frame in enumerate(SENSOR_FRAMES):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = frame
            t.transform.translation.x = SENSOR_FWD
            t.transform.translation.y = SENSOR_LAT[i]
            t.transform.translation.z = 0.0
            yaw = math.radians(SENSOR_DEG[i])
            t.transform.rotation = _yaw_quaternion(yaw)
            transforms.append(t)

        # base_scan frame at base_link origin
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'base_scan'
        t.transform.rotation = _yaw_quaternion(0.0)
        transforms.append(t)

        self._static_tf.sendTransform(transforms)

    # ══════════════════════════════════════════════════════════════════
    #  Timer callback — drain queues, publish
    # ══════════════════════════════════════════════════════════════════

    def _timer_cb(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # ── Status ────────────────────────────────────────────────────
        status_msg = String()
        if not self._client.connected:
            status_msg.data = 'DISCONNECTED'
            self._pub_status.publish(status_msg)
            return

        # ── Drain responses ───────────────────────────────────────────
        while not self._client.response_queue.empty():
            try:
                resp = self._client.response_queue.get_nowait()
            except queue.Empty:
                break
            if resp.get('type') == 'cfg':
                self._apply_cfg(resp['params'])
            elif resp.get('type') == 'car_status':
                status_msg.data = 'RUN' if resp['running'] else 'STOP'
                self._pub_status.publish(status_msg)

        # ── Drain telemetry ───────────────────────────────────────────
        frame = None
        while not self._client.telemetry_queue.empty():
            try:
                frame = self._client.telemetry_queue.get_nowait()
            except queue.Empty:
                break

        if frame is None:
            return

        # ── Publish Range messages ────────────────────────────────────
        sensors = [frame.s0, frame.s1, frame.s2, frame.s3]
        for i, pub in enumerate(self._pub_ranges):
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = SENSOR_FRAMES[i]
            r.radiation_type = Range.INFRARED
            r.field_of_view = math.radians(4.0)   # TF-Luna ~2 deg half-angle
            r.min_range = LIDAR_MIN_RANGE
            r.max_range = LIDAR_MAX_RANGE
            r.range = sensors[i] / 1000.0   # cm*10 → metres
            pub.publish(r)

        # ── Publish LaserScan (4 LiDARs combined) ─────────────────────
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_scan'
        # Angles: sensor 0 at +45 deg, sensor 3 at -45 deg
        # Order in scan: left to right = +45, +5, -5, -45
        scan.angle_min = math.radians(-45.0)
        scan.angle_max = math.radians(45.0)
        scan.angle_increment = math.radians(30.0)  # ~90/3 between 4 points
        scan.time_increment = 0.0
        scan.scan_time = 0.04   # 25 Hz
        scan.range_min = LIDAR_MIN_RANGE
        scan.range_max = LIDAR_MAX_RANGE
        # Order from angle_min to angle_max: right(-45), FR(-5), FL(+5), left(+45)
        scan.ranges = [
            sensors[3] / 1000.0,   # -45 deg (right)
            sensors[2] / 1000.0,   # ~-5 deg (front-right)
            sensors[1] / 1000.0,   # ~+5 deg (front-left)
            sensors[0] / 1000.0,   # +45 deg (left)
        ]
        self._pub_scan.publish(scan)

        # ── Publish IMU ───────────────────────────────────────────────
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        # Mark covariance as unknown (-1 in diagonal)
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        if frame.has_imu:
            yaw_rate_rad = math.radians(frame.yaw)   # deg/s → rad/s
            heading_rad = math.radians(frame.heading)
            imu.angular_velocity.z = yaw_rate_rad
            imu.orientation = _yaw_quaternion(heading_rad)
            imu.orientation_covariance[0] = 0.01
            self._last_yaw_rate = yaw_rate_rad
        else:
            self._last_yaw_rate = 0.0

        self._pub_imu.publish(imu)

        # ── Dead-reckoning odometry ───────────────────────────────────
        dt = 0.04   # 25 Hz telemetry
        speed = frame.speed
        self._last_speed = speed

        if frame.has_imu:
            heading_rad = math.radians(frame.heading)
            self._odom_heading = heading_rad
        else:
            self._odom_heading += self._last_yaw_rate * dt

        self._odom_x += speed * math.cos(self._odom_heading) * dt
        self._odom_y += speed * math.sin(self._odom_heading) * dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation = _yaw_quaternion(self._odom_heading)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = self._last_yaw_rate
        self._pub_odom.publish(odom)

        # ── Dynamic TF: odom → base_link ─────────────────────────────
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.rotation = _yaw_quaternion(self._odom_heading)
        self._dynamic_tf.sendTransform(tf)

    # ══════════════════════════════════════════════════════════════════
    #  Apply received $CFG to node parameters
    # ══════════════════════════════════════════════════════════════════

    def _apply_cfg(self, params: dict):
        """Update node parameters from received $CFG response."""
        param_list = []
        for key, val in params.items():
            if key in READONLY_KEYS:
                continue
            if key not in DEFAULTS:
                continue
            if key in FLOAT_KEYS:
                param_list.append(Parameter(key, Parameter.Type.DOUBLE, float(val)))
            else:
                param_list.append(Parameter(key, Parameter.Type.INTEGER, int(val)))
        if param_list:
            self.set_parameters(param_list)

    # ══════════════════════════════════════════════════════════════════
    #  Subscriber callbacks
    # ══════════════════════════════════════════════════════════════════

    def _cmd_vel_cb(self, msg: Twist):
        if not self._client.connected:
            return
        speed = msg.linear.x
        # Clamp speed to [0, SPD1]
        spd1 = self.get_parameter('SPD1').value
        speed = max(0.0, min(speed, spd1))

        # Map angular.z to steer: normalize to -1..+1, then scale to -1000..+1000
        # Positive angular.z = turn left = negative steer command
        max_yaw_rate = 2.0   # rad/s at full lock (approximate)
        norm = max(-1.0, min(1.0, -msg.angular.z / max_yaw_rate))
        steer = int(norm * 1000)

        self._last_cmd_steer = steer
        self._last_cmd_speed = speed
        self._client.send_command(encode_drv(steer, speed))

    def _throttle_cb(self, msg: Float32):
        if not self._client.connected:
            return
        speed = max(0.0, float(msg.data))
        self._last_cmd_speed = speed
        self._client.send_command(
            encode_drv(self._last_cmd_steer, speed))

    def _steering_cb(self, msg: Float32):
        if not self._client.connected:
            return
        norm = max(-1.0, min(1.0, float(msg.data)))
        steer = int(norm * 1000)
        self._last_cmd_steer = steer
        self._client.send_command(
            encode_drv(steer, self._last_cmd_speed))

    # ══════════════════════════════════════════════════════════════════
    #  Service callbacks
    # ══════════════════════════════════════════════════════════════════

    def _srv_connect(self, req, resp):
        ok, msg = self._do_connect()
        resp.success = ok
        resp.message = msg
        return resp

    def _srv_disconnect(self, req, resp):
        self._client.disconnect()
        resp.success = True
        resp.message = 'Disconnected'
        return resp

    def _srv_ping(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_ping())
        resp.success = r.get('type') == 'pong'
        resp.message = 'PONG' if resp.success else 'No response'
        return resp

    def _srv_start(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_start())
        resp.success = r.get('type') == 'ack'
        resp.message = 'Started' if resp.success else str(r)
        return resp

    def _srv_stop(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_stop())
        resp.success = r.get('type') == 'ack'
        resp.message = 'Stopped' if resp.success else str(r)
        return resp

    def _srv_save(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_save())
        resp.success = r.get('type') == 'ack'
        resp.message = 'Saved' if resp.success else str(r)
        return resp

    def _srv_load(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_load())
        resp.success = r.get('type') == 'ack'
        resp.message = 'Loaded' if resp.success else str(r)
        return resp

    def _srv_reset(self, req, resp):
        if not self._client.connected:
            resp.success = False
            resp.message = 'Not connected'
            return resp
        r = self._send_and_wait(encode_reset())
        resp.success = r.get('type') == 'ack'
        resp.message = 'Reset' if resp.success else str(r)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = UmbreonBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
