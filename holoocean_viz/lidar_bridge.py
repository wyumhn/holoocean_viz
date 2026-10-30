#!/usr/bin/env python3
"""
LiDAR bridge from HoloOcean to LiDAR-inertial SLAM (FAST-LIO2 / LIO-SAM).

HoloOcean's RaycastLidar returns points in the LiDAR frame (x forward, y left,
z up, rotating with the vehicle; the docstring says world coordinates, but
HoloOcean 2.3.0 does not), all float32 and without per-point time, and only on
capture ticks. This node turns the simulator output into what a real
Velodyne-style LiDAR + IMU would provide:

  * points: re-packed as x, y, z, intensity (float32), ring (uint16), time (float32,
    seconds since scan start). Captures are accumulated until one rotation
    (scan_period) is covered, so each published cloud is one full scan.
  * IMU: republished in <agent>/imu_link. LIO-SAM needs a 9-axis IMU, so the
    orientation can be filled from ground truth (imu_orientation).
  * TF: static <agent>/base_link -> <agent>/lidar_link and -> <agent>/imu_link,
    ground truth <world_frame> -> <ground_truth_frame>, and optionally a static
    <world_frame> -> <slam_origin_frame> at the initial pose so the SLAM output
    can be overlaid on ground truth in RViz.
  * ground-truth Path for comparison.

base_link is the agent COM (where DynamicsSensor with UseCOM and the COM-mounted
IMU live), so the mount offsets are the "location"/"rotation" values written in
the scenario JSON for the LiDAR and IMU.
"""
from collections import OrderedDict, deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Velodyne point layout expected by FAST-LIO2 (lidar_type 2) and LIO-SAM (sensor: velodyne)
VELODYNE_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': [np.float32, np.float32, np.float32, np.float32, np.uint16, np.float32],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})
VELODYNE_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
]

# HoloOcean RaycastLidar row layout
HOLO_FIELDS = ('x', 'y', 'z', 'intensity', 'ring')

# Every point gets at least this time offset. FAST-LIO2 treats a cloud whose last
# point has time == 0 as "no per-point time" and invents one from the azimuth,
# which would wrongly deskew a scan the simulator cast from a single pose.
MIN_POINT_TIME = 1e-6


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _sec_to_stamp(t, msg_stamp):
    ns = int(round(t * 1e9))
    msg_stamp.sec, msg_stamp.nanosec = divmod(ns, 1000000000)
    return msg_stamp


def _make_transform(stamp, parent, child, xyz, quat_xyzw):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    t.transform.rotation.x = float(quat_xyzw[0])
    t.transform.rotation.y = float(quat_xyzw[1])
    t.transform.rotation.z = float(quat_xyzw[2])
    t.transform.rotation.w = float(quat_xyzw[3])
    return t


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')

        self.declare_parameter('agent_name', 'auv0')
        self.declare_parameter('lidar_topic', '')           # default: /<agent>/RaycastLidar
        self.declare_parameter('odom_topic', '')            # default: /<agent>/DynamicsSensorOdom
        self.declare_parameter('imu_topic', '')             # default: /<agent>/IMUSensor
        self.declare_parameter('points_out_topic', '')      # default: /<agent>/lidar/points
        self.declare_parameter('imu_out_topic', '')         # default: /<agent>/imu/data
        self.declare_parameter('world_frame', 'map')
        # 'sensor': already in the LiDAR frame (HoloOcean 2.3.0), only re-pack and add time
        # 'world': world coordinates, converted with the ground-truth pose at the same tick
        self.declare_parameter('input_frame', 'sensor')
        # Mount offsets relative to the COM, same values as the scenario JSON
        self.declare_parameter('lidar_xyz', [0.0, 0.0, 0.2])
        self.declare_parameter('lidar_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('scan_period', 0.1)          # 1 / RotationFrequency
        self.declare_parameter('min_range', 0.3)            # drop hits on the vehicle itself
        self.declare_parameter('max_range', 100.0)
        # 'ground_truth': fill Imu.orientation from DynamicsSensorOdom (needed by LIO-SAM)
        # 'none': keep it empty (orientation_covariance[0] = -1), enough for FAST-LIO2
        self.declare_parameter('imu_orientation', 'ground_truth')
        self.declare_parameter('publish_ground_truth', True)
        # Child frame of the ground-truth TF. Default <agent>/base_link_gt; set it to
        # <agent>/base_link when no SLAM publishes base_link (FAST-LIO2 uses its own 'body').
        self.declare_parameter('ground_truth_frame', '')
        # Static TF <world_frame> -> <slam_origin_frame> at the initial pose of
        # slam_origin_link ('imu' or 'lidar'). '' disables.
        # FAST-LIO2: 'camera_init' with 'imu'. LIO-SAM: its odometryFrame with 'lidar', no rotation.
        self.declare_parameter('slam_origin_frame', '')
        self.declare_parameter('slam_origin_link', 'imu')
        # False: origin keeps only the position and stays aligned with the world axes
        # (LIO-SAM with useImuHeadingInitialization starts gravity- and heading-aligned)
        self.declare_parameter('slam_origin_rotation', True)

        agent = self.get_parameter('agent_name').value
        lidar_topic = self.get_parameter('lidar_topic').value or f'/{agent}/RaycastLidar'
        odom_topic = self.get_parameter('odom_topic').value or f'/{agent}/DynamicsSensorOdom'
        imu_topic = self.get_parameter('imu_topic').value or f'/{agent}/IMUSensor'
        points_out = self.get_parameter('points_out_topic').value or f'/{agent}/lidar/points'
        imu_out = self.get_parameter('imu_out_topic').value or f'/{agent}/imu/data'

        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = f'{agent}/base_link'
        self.lidar_frame = f'{agent}/lidar_link'
        self.imu_frame = f'{agent}/imu_link'
        self.gt_frame = self.get_parameter('ground_truth_frame').value or f'{agent}/base_link_gt'

        self.input_frame = self.get_parameter('input_frame').value
        if self.input_frame not in ('world', 'sensor'):
            raise ValueError("input_frame must be 'world' or 'sensor'")
        self.scan_period = float(self.get_parameter('scan_period').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.imu_orientation = self.get_parameter('imu_orientation').value
        self.publish_gt = self.get_parameter('publish_ground_truth').value
        self.slam_origin_frame = self.get_parameter('slam_origin_frame').value
        self.slam_origin_link = self.get_parameter('slam_origin_link').value
        self.slam_origin_rotation = self.get_parameter('slam_origin_rotation').value

        # base_link -> sensor mounts
        self.t_bl = np.array(self.get_parameter('lidar_xyz').value, dtype=float)
        self.r_bl = Rotation.from_euler('xyz', self.get_parameter('lidar_rpy_deg').value, degrees=True)
        self.t_bi = np.array(self.get_parameter('imu_xyz').value, dtype=float)
        self.r_bi = Rotation.from_euler('xyz', self.get_parameter('imu_rpy_deg').value, degrees=True)

        # Ground-truth poses by stamp (seconds), to match LiDAR / IMU messages of the same tick
        self.poses = OrderedDict()
        self.max_poses = 200

        # Scan accumulation
        self.scan_chunks = []
        self.scan_start = None
        self.last_capture = None
        self.capture_dt = None
        self.origin_sent = False
        self.warned_no_pose = False

        # Messages whose ground-truth pose has not arrived yet (subscriptions are
        # independent, so the odom of a tick can come after its LiDAR / IMU)
        self.pending_lidar = deque()
        self.pending_imu = deque()
        self.max_pending = 50

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_tfs = [
            _make_transform(self.get_clock().now().to_msg(), self.base_frame, self.lidar_frame,
                            self.t_bl, self.r_bl.as_quat()),
            _make_transform(self.get_clock().now().to_msg(), self.base_frame, self.imu_frame,
                            self.t_bi, self.r_bi.as_quat()),
        ]
        self.static_broadcaster.sendTransform(self.static_tfs)

        self.points_pub = self.create_publisher(PointCloud2, points_out, 10)
        self.imu_pub = self.create_publisher(Imu, imu_out, 200)
        if self.publish_gt:
            self.gt_path = Path()
            self.gt_path.header.frame_id = self.world_frame
            self.gt_odom_pub = self.create_publisher(Odometry, f'/{agent}/ground_truth/odom', 10)
            self.gt_path_pub = self.create_publisher(Path, f'/{agent}/ground_truth/path', 10)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 200)
        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 200)

        self.get_logger().info(
            f'lidar_bridge: {lidar_topic} ({self.input_frame} coords) -> {points_out} [{self.lidar_frame}], '
            f'{imu_topic} -> {imu_out} [{self.imu_frame}], pose from {odom_topic}')

    # ---------------------------------------------------------------- ground truth
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = _stamp_to_sec(msg.header.stamp)
        pose = (np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w]))

        self.poses[round(t, 6)] = pose
        while len(self.poses) > self.max_poses:
            self.poses.popitem(last=False)

        if not self.origin_sent and self.slam_origin_frame:
            self._send_slam_origin(pose)

        if self.publish_gt:
            self._publish_ground_truth(msg, pose)

        self._flush_pending()

    def _flush_pending(self):
        while self.pending_lidar and self._process_lidar(self.pending_lidar[0]):
            self.pending_lidar.popleft()
        while self.pending_imu and self._process_imu(self.pending_imu[0]):
            self.pending_imu.popleft()
        if len(self.pending_lidar) > self.max_pending or len(self.pending_imu) > self.max_pending:
            if not self.warned_no_pose:
                self.get_logger().warn('No ground-truth pose for LiDAR / IMU stamps; dropping data. '
                                       'Is DynamicsSensor ros_publish enabled?')
                self.warned_no_pose = True
            while len(self.pending_lidar) > self.max_pending:
                self.pending_lidar.popleft()
            while len(self.pending_imu) > self.max_pending:
                self.pending_imu.popleft()

    def _lookup_pose(self, t):
        """Ground-truth pose of the tick at time t, or None if it may still arrive."""
        pose = self.poses.get(round(t, 6))
        if pose is not None:
            return pose
        # A newer pose already arrived, so this tick's pose will never come
        # (e.g. DynamicsSensor at a lower rate): use the closest one instead
        if self.poses and next(reversed(self.poses)) > t:
            key = min(self.poses, key=lambda k: abs(k - t))
            return self.poses[key]
        return None

    def _send_slam_origin(self, pose):
        t_wb, r_wb = pose
        if self.slam_origin_link == 'lidar':
            t_ws, r_ws = t_wb + r_wb.apply(self.t_bl), r_wb * self.r_bl
        else:
            t_ws, r_ws = t_wb + r_wb.apply(self.t_bi), r_wb * self.r_bi
        if not self.slam_origin_rotation:
            r_ws = Rotation.identity()
        origin = _make_transform(self.get_clock().now().to_msg(), self.world_frame, self.slam_origin_frame,
                                 t_ws, r_ws.as_quat())
        self.static_broadcaster.sendTransform(self.static_tfs + [origin])
        self.origin_sent = True
        self.get_logger().info(f'static TF {self.world_frame} -> {self.slam_origin_frame} set at initial pose')

    def _publish_ground_truth(self, msg, pose):
        t_wb, r_wb = pose
        self.tf_broadcaster.sendTransform(
            _make_transform(msg.header.stamp, self.world_frame, self.gt_frame, t_wb, r_wb.as_quat()))

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.gt_frame
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.gt_odom_pub.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = msg.pose.pose
        self.gt_path.header.stamp = msg.header.stamp
        self.gt_path.poses.append(ps)
        self.gt_path_pub.publish(self.gt_path)

    # ---------------------------------------------------------------- IMU
    def _imu_cb(self, msg: Imu):
        if self.pending_imu or not self._process_imu(msg):
            self.pending_imu.append(msg)
            self._flush_pending()

    def _process_imu(self, msg):
        """Publish one IMU sample. Returns False if its pose is not available yet."""
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.imu_frame
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        out.orientation_covariance[0] = -1.0
        if self.imu_orientation == 'ground_truth':
            pose = self._lookup_pose(_stamp_to_sec(msg.header.stamp))
            if pose is None:
                return False  # LIO-SAM needs every IMU sample to carry an orientation
            q = (pose[1] * self.r_bi).as_quat()
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = (float(v) for v in q)
            out.orientation_covariance = [0.0] * 9

        self.imu_pub.publish(out)
        return True

    # ---------------------------------------------------------------- LiDAR
    def _lidar_cb(self, msg: PointCloud2):
        if msg.width * msg.height == 0:
            # Tick without a capture (sensor Hz < ticks_per_sec): not part of the scan timing
            return
        if self.pending_lidar or not self._process_lidar(msg):
            self.pending_lidar.append(msg)
            self._flush_pending()

    def _process_lidar(self, msg):
        """Add one capture to the current scan. Returns False if its pose is not available yet."""
        t = _stamp_to_sec(msg.header.stamp)
        pose = None
        if self.input_frame == 'world':
            pose = self._lookup_pose(t)
            if pose is None:
                return False

        if self.last_capture is not None:
            dt = t - self.last_capture
            if dt <= 0.0 or dt > 1.0:
                # Simulator reset or time jump: start over
                self.scan_chunks, self.scan_start = [], None
            else:
                self.capture_dt = dt
        self.last_capture = t

        points = self._read_points(msg)
        if points is None:
            return True

        if pose is not None:
            t_wb, r_wb = pose
            # world -> LiDAR: p_l = R_wl^T (p_w - t_wl)
            r_wl = r_wb * self.r_bl
            t_wl = t_wb + r_wb.apply(self.t_bl)
            xyz = r_wl.inv().apply(points[:, :3] - t_wl)
        else:
            xyz = points[:, :3]

        rng = np.linalg.norm(xyz, axis=1)
        keep = (rng >= self.min_range) & (rng <= self.max_range)

        if self.scan_start is None:
            self.scan_start = t
        chunk = np.empty(int(keep.sum()), dtype=VELODYNE_DTYPE)
        chunk['x'], chunk['y'], chunk['z'] = xyz[keep, 0], xyz[keep, 1], xyz[keep, 2]
        chunk['intensity'] = points[keep, 3]
        chunk['ring'] = np.clip(np.rint(points[keep, 4]), 0, 65535).astype(np.uint16)
        chunk['time'] = max(t - self.scan_start, MIN_POINT_TIME)
        self.scan_chunks.append(chunk)

        # Publish once the captures cover one rotation
        dt = self.capture_dt if self.capture_dt is not None else self.scan_period
        if (t - self.scan_start) + dt >= self.scan_period - 1e-6:
            self._publish_scan()
        return True

    def _read_points(self, msg):
        names = [f.name for f in msg.fields]
        if any(n not in names for n in HOLO_FIELDS):
            self.get_logger().error(f'LiDAR cloud is missing fields {HOLO_FIELDS}, got {names}')
            return None
        offsets = {f.name: f.offset for f in msg.fields}
        dtype = np.dtype({'names': list(HOLO_FIELDS),
                          'formats': [np.float32] * len(HOLO_FIELDS),
                          'offsets': [offsets[n] for n in HOLO_FIELDS],
                          'itemsize': msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
        return np.stack([raw[n] for n in HOLO_FIELDS], axis=1).astype(np.float64)

    def _publish_scan(self):
        cloud = np.concatenate(self.scan_chunks) if self.scan_chunks else np.empty(0, VELODYNE_DTYPE)
        stamp_sec = self.scan_start
        self.scan_chunks, self.scan_start = [], None
        if cloud.size == 0:
            return

        out = PointCloud2()
        _sec_to_stamp(stamp_sec, out.header.stamp)   # scan start; per-point time is relative to it
        out.header.frame_id = self.lidar_frame
        out.height = 1
        out.width = cloud.shape[0]
        out.fields = VELODYNE_FIELDS
        out.is_bigendian = False
        out.point_step = VELODYNE_DTYPE.itemsize
        out.row_step = out.point_step * out.width
        out.is_dense = True                           # LIO-SAM refuses non-dense clouds
        out.data = cloud.tobytes()
        self.points_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
