#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Pursuit follower for differential-drive robots (ROS 2) — with *dynamic* RViz path publishing.
Now with near-goal braking funnel, lookahead clamp near goal, and sticky stop + final align.
"""

import os
import math
import glob
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


# ----------------------- small utils -----------------------
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


# ---------------------- main follower ----------------------
class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('pure_pursuit_follower')

        # --- Paths & frames ---
        self.declare_parameter('package_name', 'autonomous_explorer')
        self.declare_parameter('path_folder_rel', 'path')
        self.declare_parameter('path_folder_abs', '')
        self.declare_parameter('path_file', '')  # explicit CSV/YAML; empty => newest in folder

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Pose source / timeouts
        self.declare_parameter('pose_source', 'amcl')  # 'amcl' or 'tf'
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('pose_stale_timeout_s', 0.50)
        self.declare_parameter('tf_timeout_s', 0.25)

        # --- Kinematics & limits ---
        self.declare_parameter('v_cruise', 5.0)        # desired cruise speed [m/s]
        self.declare_parameter('v_min', 0.8)           # floor to avoid stalling (except near goal if allowed)
        self.declare_parameter('w_max', 4.0)           # max yaw rate [rad/s]
        self.declare_parameter('accel_max', 3.5)       # [m/s^2]
        self.declare_parameter('decel_max', 2.8)       # [m/s^2]

        # Cornering slowdown (gentle; ON by default)
        self.declare_parameter('allow_corner_slowdown', True)
        self.declare_parameter('a_lat_max', 1.6)       # m/s^2, used only if slowdown enabled
        self.declare_parameter('min_corner_speed', 0.5)# clamp so we don't crawl
        self.declare_parameter('kappa_smooth_alpha', 0.3)

        # Lookahead scheduling
        self.declare_parameter('Ld_min', 0.8)
        self.declare_parameter('Ld_max', 3.0)
        self.declare_parameter('Ld_gain', 1.2)         # scales with speed
        self.declare_parameter('Ld_use_curvature', False)
        self.declare_parameter('Ld_curv_scale', 1.2)   # if enabled: Ld ≈ scale / sqrt(kappa_preview)
        self.declare_parameter('kpreview_m', 2.0)

        # Goal handling
        self.declare_parameter('stop_at_goal', True)
        self.declare_parameter('goal_pos_tol', 0.2)    # m
        self.declare_parameter('goal_yaw_tol_deg', 20.0)

        # >>> New near-goal heuristics <<<
        self.declare_parameter('goal_slowdown_dist', 5.0)         # m: start easing down this far out
        self.declare_parameter('allow_below_vmin_near_goal', True)# let v drop below v_min inside slowdown_dist
        self.declare_parameter('Ld_goal_scale', 0.6)              # Ld <= Ld_goal_scale * remaining
        self.declare_parameter('goal_stop_margin', 1.5)          # m: stop slightly before goal
        self.declare_parameter('goal_dwell_s', 1.0)               # s: hold still after stopping
        self.declare_parameter('goal_final_align', True)          # after stop, align yaw to path end

        # Path processing
        self.declare_parameter('resample_ds', 0.10)     # meters; 0 disables resampling

        # Optional front safety using scan
        self.declare_parameter('use_scan_gate', False)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_width_deg', 30.0)
        self.declare_parameter('front_stop', 0.35)
        self.declare_parameter('front_clear', 0.80)

        # ALIGN mode (turn-in-place to face route before starting)
        self.declare_parameter('align_to_path_heading', True)
        self.declare_parameter('align_yaw_tol_deg', 6.0)     # allowed heading error to finish align
        self.declare_parameter('align_timeout_s', 10.0)      # safety cap; 0 disables timeout
        self.declare_parameter('align_kp', 1.2)              # P gain for yaw error
        self.declare_parameter('align_kd', 0.05)             # D gain for yaw error
        self.declare_parameter('align_preview_m', 1.0)       # heading of path a bit ahead of nearest s

        # IO, debug, rate
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pub_debug', True)
        self.declare_parameter('rate_hz', 50.0)

        # --- Read params ---
        gp = self.get_parameter
        self.package_name   = gp('package_name').value
        self.path_folder_rel= gp('path_folder_rel').value
        self.path_folder_abs= gp('path_folder_abs').value
        self.path_file      = gp('path_file').value

        self.global_frame   = gp('global_frame').value
        self.base_frame     = gp('base_frame').value

        self.pose_source    = gp('pose_source').value
        self.amcl_topic     = gp('amcl_pose_topic').value
        self.pose_stale_t   = float(gp('pose_stale_timeout_s').value)
        self.tf_timeout     = float(gp('tf_timeout_s').value)

        self.v_cruise       = float(gp('v_cruise').value)
        self.v_min          = float(gp('v_min').value)
        self.w_max          = float(gp('w_max').value)
        self.accel_max      = float(gp('accel_max').value)
        self.decel_max      = float(gp('decel_max').value)

        self.allow_corner_slowdown = bool(gp('allow_corner_slowdown').value)
        self.a_lat_max      = max(1e-3, float(gp('a_lat_max').value))
        self.min_corner_speed = float(gp('min_corner_speed').value)
        self.kappa_alpha    = float(gp('kappa_smooth_alpha').value)

        self.Ld_min         = float(gp('Ld_min').value)
        self.Ld_max         = float(gp('Ld_max').value)
        self.Ld_gain        = float(gp('Ld_gain').value)
        self.Ld_use_curv    = bool(gp('Ld_use_curvature').value)
        self.Ld_curv_scale  = float(gp('Ld_curv_scale').value)
        self.kpreview_m     = float(gp('kpreview_m').value)

        self.stop_at_goal   = bool(gp('stop_at_goal').value)
        self.goal_pos_tol   = float(gp('goal_pos_tol').value)
        self.goal_yaw_tol   = math.radians(float(gp('goal_yaw_tol_deg').value))

        # New goal heuristics
        self.goal_slowdown_dist = float(gp('goal_slowdown_dist').value)
        self.allow_below_vmin_near_goal = bool(gp('allow_below_vmin_near_goal').value)
        self.Ld_goal_scale = float(gp('Ld_goal_scale').value)
        self.goal_stop_margin = float(gp('goal_stop_margin').value)
        self.goal_dwell_s = float(gp('goal_dwell_s').value)
        self.goal_final_align = bool(gp('goal_final_align').value)

        self.resample_ds    = float(gp('resample_ds').value)

        self.use_scan_gate  = bool(gp('use_scan_gate').value)
        self.scan_topic     = gp('scan_topic').value
        self.front_width    = math.radians(float(gp('front_width_deg').value))
        self.front_stop     = float(gp('front_stop').value)
        self.front_clear    = float(gp('front_clear').value)

        self.align_enable   = bool(gp('align_to_path_heading').value)
        self.align_tol      = math.radians(float(gp('align_yaw_tol_deg').value))
        self.align_timeout  = float(gp('align_timeout_s').value)
        self.align_kp       = float(gp('align_kp').value)
        self.align_kd       = float(gp('align_kd').value)
        self.align_preview  = float(gp('align_preview_m').value)

        self.cmd_vel_topic  = gp('cmd_vel_topic').value
        self.pub_debug      = bool(gp('pub_debug').value)
        self.rate_hz        = float(gp('rate_hz').value)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # RViz debug pubs
        self.path_full_pub      = self.create_publisher(Path, '/pp_path_full', 1)
        self.path_done_pub      = self.create_publisher(Path, '/pp_path_done', 1)
        self.path_remaining_pub = self.create_publisher(Path, '/pp_path_remaining', 1)
        self.goal_pub           = self.create_publisher(PoseStamped, '/pp_goal', 1)
        self.lookahead_pub      = self.create_publisher(PoseStamped, '/pp_lookahead', 1) if self.pub_debug else None
        self.marker_pub         = self.create_publisher(Marker, '/pp_markers', 1) if self.pub_debug else None

        # Scan (optional)
        qos_scan = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.last_scan: Optional[LaserScan] = None
        if self.use_scan_gate:
            self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_scan)

        # Pose from AMCL (reliable QoS)
        qos_amcl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.last_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.last_amcl_time: Optional[Time] = None
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic, self._amcl_cb, qos_amcl)

        # TF fallback
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Load & prep path ---
        pts = self._load_waypoints()
        if len(pts) < 2:
            raise RuntimeError("Path has <2 waypoints.")
        xy_raw = np.asarray(pts, dtype=np.float32)
        self.path_xy = self._resample_path(xy_raw, self.resample_ds) if self.resample_ds > 1e-6 else xy_raw

        # Geometry along path
        self.s, self.kappa_path = self._arc_length_and_curvature(self.path_xy)

        # State
        self.idx_near = 0
        self.v_prev = 0.0
        self.kappa_f = 0.0
        self.prev_brg_err = 0.0

        # ALIGN state machine
        self.mode = 'ALIGN' if self.align_enable else 'TRACK'
        self.align_start_time = self.get_clock().now()
        self.prev_align_err = 0.0

        # Goal latch
        self.goal_stop_latch = False
        self.goal_dwell_until: Optional[Time] = None

        # Publish static/full path & goal once up front
        self._publish_full_path_once()
        self._publish_goal_every_cycle = True  # publish goal each loop for convenience in RViz

        # Loop
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self._loop)
        self.get_logger().info(
            f"PP follower up. waypoints={len(self.path_xy)}  cruise={self.v_cruise} m/s  "
            f"corner_slowdown={'ON' if self.allow_corner_slowdown else 'OFF'}  "
            f"mode={self.mode}"
        )

    # ---------------- Path loading ----------------
    def _resolve_path_dir(self) -> str:
        p = str(self.path_folder_abs or '').strip()
        if p:
            return os.path.abspath(p)
        if get_package_share_directory is not None:
            try:
                share = get_package_share_directory(self.package_name)
                install_pkg = os.path.dirname(os.path.dirname(share))
                ws_root = os.path.dirname(os.path.dirname(install_pkg))
                cand_src = os.path.join(ws_root, 'src', self.package_name)
                return os.path.join(cand_src if os.path.isdir(cand_src) else share, self.path_folder_rel)
            except Exception:
                pass
        return os.path.join(os.getcwd(), self.path_folder_rel)

    def _newest_path_file(self, folder: str) -> str:
        cands = glob.glob(os.path.join(folder, '*.csv')) + \
                glob.glob(os.path.join(folder, '*.yaml')) + \
                glob.glob(os.path.join(folder, '*.yml'))
        if not cands:
            raise FileNotFoundError(f'No path files in {folder}')
        cands.sort(key=os.path.getmtime, reverse=True)
        return cands[0]

    def _load_waypoints(self) -> List[Tuple[float, float]]:
        f = str(self.path_file).strip()
        if not f:
            folder = self._resolve_path_dir()
            os.makedirs(folder, exist_ok=True)
            f = self._newest_path_file(folder)
        f = os.path.abspath(f)
        if f.lower().endswith('.csv'):
            pts = []
            with open(f, 'r') as fp:
                for ln in fp:
                    s = ln.strip()
                    if not s or s.startswith('#'): continue
                    parts = s.split(',')
                    if len(parts) >= 2:
                        try:
                            x = float(parts[0]); y = float(parts[1])
                            pts.append((x, y))
                        except Exception:
                            pass
            self.get_logger().info(f"Loaded CSV path: {f} (N={len(pts)})")
            return pts
        else:
            try:
                import yaml
                data = yaml.safe_load(open(f, 'r'))
                wps = data.get('waypoints', [])
                pts = [(float(d['x']), float(d['y'])) for d in wps if ('x' in d and 'y' in d)]
            except Exception:
                pts = []
                with open(f, 'r') as fp:
                    for ln in fp:
                        if '- {' in ln and 'x:' in ln and 'y:' in ln:
                            try:
                                seg = ln.split('{',1)[1].split('}',1)[0]
                                items = dict([kv.strip().split(':') for kv in seg.split(',')])
                                pts.append((float(items['x']), float(items['y'])))
                            except Exception:
                                pass
            self.get_logger().info(f"Loaded YAML path: {f} (N={len(pts)})")
            return pts

    # --------------- Path geometry ----------------
    def _arc_length_and_curvature(self, xy: np.ndarray):
        d = np.linalg.norm(xy[1:] - xy[:-1], axis=1)
        s = np.zeros(len(xy), dtype=np.float32)
        s[1:] = np.cumsum(d, dtype=np.float32)

        kappa = np.zeros(len(xy), dtype=np.float32)
        for i in range(1, len(xy)-1):
            p0, p1, p2 = xy[i-1], xy[i], xy[i+1]
            a = np.linalg.norm(p1-p0); b = np.linalg.norm(p2-p1); c = np.linalg.norm(p2-p0)
            area2 = abs(np.cross(p1-p0, p2-p0))
            if a*b*c < 1e-6 or area2 < 1e-9:
                kappa[i] = 0.0
            else:
                kappa[i] = 2.0 * area2 / (a*b*c)
        kappa[0]  = kappa[1]
        kappa[-1] = kappa[-2]
        return s, kappa

    def _resample_path(self, xy: np.ndarray, ds: float) -> np.ndarray:
        if ds <= 0.0 or len(xy) < 2: return xy.copy()
        d = np.linalg.norm(xy[1:] - xy[:-1], axis=1)
        s = np.zeros(len(xy), dtype=np.float32)
        s[1:] = np.cumsum(d, dtype=np.float32)
        total = float(s[-1])
        if total < 1e-6: return xy.copy()
        n_pts = max(2, int(round(total/ds)) + 1)
        s_new = np.linspace(0.0, total, n_pts, dtype=np.float32)
        xy_new = []
        for sv in s_new:
            if sv <= s[0]:
                xy_new.append(xy[0]); continue
            if sv >= s[-1]:
                xy_new.append(xy[-1]); continue
            j = int(np.searchsorted(s, sv) - 1)
            j = max(0, min(j, len(s)-2))
            t = (sv - s[j]) / max(1e-9, (s[j+1]-s[j]))
            p = (1.0-t)*xy[j] + t*xy[j+1]
            xy_new.append(p)
        return np.asarray(xy_new, dtype=np.float32)

    # ------------------- Subscriptions -------------------
    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg
        self.last_amcl_time = self.get_clock().now()

    # ------------------- Pose getters -------------------
    def _pose_from_amcl(self) -> Optional[Tuple[float,float,float]]:
        if self.last_amcl_pose is None or self.last_amcl_time is None:
            return None
        if (self.get_clock().now() - self.last_amcl_time) > Duration(seconds=self.pose_stale_t):
            return None
        p = self.last_amcl_pose.pose.pose.position
        q = self.last_amcl_pose.pose.pose.orientation
        return float(p.x), float(p.y), yaw_from_quat(q.x, q.y, q.z, q.w)

    def _pose_from_tf(self) -> Optional[Tuple[float,float,float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time(),
                                                 timeout=Duration(seconds=self.tf_timeout))
            t = tf.transform.translation
            q = tf.transform.rotation
            return float(t.x), float(t.y), yaw_from_quat(q.x, q.y, q.z, q.w)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _robot_pose(self) -> Optional[Tuple[float,float,float]]:
        if self.pose_source == 'amcl':
            p = self._pose_from_amcl()
            if p is not None:
                return p
            return self._pose_from_tf()
        else:
            p = self._pose_from_tf()
            if p is not None:
                return p
            return self._pose_from_amcl()

    # ------------------- Helpers on path -------------------
    def _nearest_index(self, x, y, hint_idx: int) -> int:
        N = len(self.path_xy)
        w = int(25 + 60 * clamp(self.v_prev / max(1e-6, max(self.v_cruise, 0.1)), 0.0, 1.0))
        lo = max(0, hint_idx - w)
        hi = min(N-1, hint_idx + w)
        seg = self.path_xy[lo:hi+1]
        d2 = np.sum((seg - np.array([x,y]))**2, axis=1)
        k = int(np.argmin(d2))
        return lo + k

    def _interp_at_s(self, s_query: float) -> Tuple[float,float,int]:
        s = self.s
        if s_query <= s[0]:
            return float(self.path_xy[0,0]), float(self.path_xy[0,1]), 0
        if s_query >= s[-1]:
            return float(self.path_xy[-1,0]), float(self.path_xy[-1,1]), len(s)-1
        i = int(np.searchsorted(s, s_query) - 1)
        i = max(0, min(i, len(s)-2))
        t = (s_query - s[i]) / max(1e-9, (s[i+1]-s[i]))
        p = (1.0-t)*self.path_xy[i] + t*self.path_xy[i+1]
        return float(p[0]), float(p[1]), i

    def _path_heading_at_s(self, s_query: float) -> float:
        x, y, i = self._interp_at_s(s_query)
        if i >= len(self.path_xy)-1: i = len(self.path_xy)-2
        d = self.path_xy[i+1] - self.path_xy[i]
        return math.atan2(float(d[1]), float(d[0])) if np.linalg.norm(d) > 1e-9 else 0.0

    def _goal_heading(self) -> float:
        # heading of last segment
        if len(self.path_xy) >= 2:
            d = self.path_xy[-1] - self.path_xy[-2]
            if np.linalg.norm(d) > 1e-9:
                return math.atan2(float(d[1]), float(d[0]))
        return 0.0

    def _kappa_preview_max(self, idx_start: int, preview_m: float) -> float:
        s0 = float(self.s[idx_start]); s_lim = s0 + max(0.0, preview_m)
        i = idx_start; kmax = 0.0; N = len(self.s)
        while i < N and float(self.s[i]) <= s_lim:
            kmax = max(kmax, abs(float(self.kappa_path[i])))
            i += 1
        return kmax

    def _front_min(self) -> float:
        if self.last_scan is None:
            return float('inf')
        msg = self.last_scan
        a0, aincr = msg.angle_min, msg.angle_increment
        n = len(msg.ranges)
        width = self.front_width
        lo = -width/2.0; hi = +width/2.0

        def idx(theta):
            off = theta - a0
            i = int(math.floor(off/aincr))
            return max(0, min(i, n-1))

        i0 = idx(lo); i1 = idx(hi)
        idxs = range(i0, i1+1) if i1 >= i0 else list(range(i0, n))+list(range(0, i1+1))
        rmin = float('inf')
        rlo = max(0.0, msg.range_min)
        rhi = msg.range_max if msg.range_max > 0.0 else 1e9
        for i in idxs:
            r = msg.ranges[i]
            if r is None or not math.isfinite(r):
                continue
            if r <= rlo or r > rhi:
                continue
            rmin = min(rmin, r)
        return rmin

    # ------------------- RViz dynamic publishing -------------------
    def _build_path_msg(self, xy: np.ndarray) -> Path:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        return path

    def _publish_full_path_once(self):
        self.path_full_pub.publish(self._build_path_msg(self.path_xy))
        # Publish goal once (we'll also refresh each loop if desired)
        self._publish_goal_pose()

    def _publish_goal_pose(self):
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(self.path_xy[-1, 0])
        ps.pose.position.y = float(self.path_xy[-1, 1])
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

    def _publish_dynamic_paths(self, idx_near: int):
        # DONE: from start up to current nearest index (include idx_near)
        idx_near = max(0, min(int(idx_near), len(self.path_xy)-1))
        done_xy = self.path_xy[:idx_near+1]
        rem_xy  = self.path_xy[idx_near:] if idx_near < len(self.path_xy) else self.path_xy[-1:]

        self.path_done_pub.publish(self._build_path_msg(done_xy))
        self.path_remaining_pub.publish(self._build_path_msg(rem_xy))

    def _pub_lookahead(self, x, y):
        if self.lookahead_pub is None: return
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        self.lookahead_pub.publish(ps)

    def _pub_marker_lookahead(self, x, y):
        if self.marker_pub is None: return
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pure_pursuit"
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.MODIFY
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.15
        m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.1
        self.marker_pub.publish(m)

    def _pub_marker_target_heading(self, x, y):
        if self.marker_pub is None: return
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pure_pursuit"
        m.id = 2
        m.type = Marker.CYLINDER
        m.action = Marker.MODIFY
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.10; m.scale.y = 0.10; m.scale.z = 0.05
        m.color.a = 0.8; m.color.r = 0.2; m.color.g = 0.4; m.color.b = 1.0
        self.marker_pub.publish(m)

    # ------------------- Main control loop -------------------
    def _loop(self):
        # Handle dwell/latched stop (hold still)
        if self.goal_stop_latch:
            if self.goal_dwell_until is None or self.get_clock().now() <= self.goal_dwell_until:
                # Optional final align while dwelling
                v = 0.0
                w = 0.0
                if self.goal_final_align:
                    pose = self._robot_pose()
                    if pose is not None:
                        _, _, ryaw = pose
                        err = wrap(self._goal_heading() - ryaw)
                        if abs(err) > self.goal_yaw_tol:
                            w = clamp(self.align_kp * err, -self.w_max, self.w_max)
                self._send(v, w)
                return
            else:
                # After dwell, keep zero and stay latched
                self._send(0.0, 0.0)
                return

        pose = self._robot_pose()
        if pose is None:
            self._send(0.0, 0.0)
            return
        rx, ry, ryaw = pose

        # nearest path index & station
        self.idx_near = self._nearest_index(rx, ry, self.idx_near)
        s_near = self.s[self.idx_near]

        # Publish dynamic RViz views
        self._publish_dynamic_paths(self.idx_near)
        if self._publish_goal_every_cycle:
            self._publish_goal_pose()

        # -------- ALIGN MODE (turn-in-place to face the route) --------
        if self.mode == 'ALIGN':
            s_des = min(self.s[-1], s_near + max(0.2, self.align_preview))
            hdg_des = self._path_heading_at_s(s_des)
            err = wrap(hdg_des - ryaw)
            d_err = (err - self.prev_align_err) / max(1e-3, self.dt)
            self.prev_align_err = err

            w_cmd = clamp(self.align_kp * err + self.align_kd * d_err, -self.w_max, self.w_max)
            self._send(0.0, w_cmd)

            aligned = abs(err) <= self.align_tol
            timed_out = (self.align_timeout > 0.0) and ((self.get_clock().now() - self.align_start_time) > Duration(seconds=self.align_timeout))
            if aligned or timed_out:
                self.mode = 'TRACK'
                self._send(0.0, 0.0)
                self.get_logger().info(f"ALIGN -> TRACK (aligned={aligned}, timeout={timed_out})")

            if self.pub_debug:
                xh, yh, _ = self._interp_at_s(s_des)
                self._pub_marker_target_heading(xh, yh)
            return  # skip tracking this cycle

        # -------- TRACK MODE (pure pursuit) --------
        # Remaining distance to goal
        remaining = max(0.0, float(self.s[-1] - s_near))

        # Lookahead scheduling (base)
        Ld_vel = self.Ld_min + self.Ld_gain * max(0.0, self.v_prev)
        if self.Ld_use_curv:
            kmax = self._kappa_preview_max(self.idx_near, self.kpreview_m)
            Ld_curv = self.Ld_curv_scale / math.sqrt(max(1e-6, kmax)) if kmax > 1e-9 else self.Ld_max
            Ld = clamp(min(Ld_vel, Ld_curv), self.Ld_min, self.Ld_max)
        else:
            Ld = clamp(Ld_vel, self.Ld_min, self.Ld_max)

        # >>> NEW: clamp lookahead near the goal so target doesn't go past goal <<<
        if remaining < self.goal_slowdown_dist:
            Ld_cap = max(0.5*self.Ld_min, self.Ld_goal_scale * remaining)  # allow smaller than Ld_min near goal
            Ld = min(Ld, max(0.05, Ld_cap))

        # target point at s + Ld
        s_target = min(self.s[-1], s_near + Ld)
        gx, gy, _ = self._interp_at_s(s_target)

        # transform target into robot frame
        dx = gx - rx; dy = gy - ry
        ca, sa = math.cos(-ryaw), math.sin(-ryaw)
        xR =  ca*dx - sa*dy
        yR =  sa*dx + ca*dy
        L = max(1e-3, math.hypot(xR, yR))

        # Pure pursuit curvature, smoothed
        kappa_raw = (2.0 * yR) / (L*L)
        self.kappa_f = (1.0 - self.kappa_alpha) * self.kappa_f + self.kappa_alpha * kappa_raw
        kappa_use = self.kappa_f

        # Path heading ahead for mild PD (keeps corners tight w/o braking)
        hdg_path = self._path_heading_at_s(min(self.s[-1], s_target + 0.5 * Ld))
        brg_err = wrap(hdg_path - ryaw)
        heading_kp = 0.4
        heading_kd = 0.02
        d_brg = (brg_err - self.prev_brg_err) / max(1e-3, self.dt)
        self.prev_brg_err = brg_err

        # Speed target base
        v_des = max(self.v_min, self.v_cruise)

        # Cornering slowdown
        if self.allow_corner_slowdown:
            if abs(kappa_use) > 1e-6:
                v_lat_cap = math.sqrt(max(0.0, self.a_lat_max / abs(kappa_use)))
                v_des = max(self.min_corner_speed, min(v_des, v_lat_cap))

        # Optional obstacle gate
        if self.use_scan_gate:
            rmin = self._front_min()
            if rmin < self.front_stop:
                v_des = 0.0
            elif rmin < self.front_clear:
                x = (rmin - self.front_stop) / max(1e-6, (self.front_clear - self.front_stop))
                v_des *= clamp(x, 0.0, 1.0)

        # >>> NEW: Smooth near-goal braking funnel (cubic ease-out) <<<
        # starts easing inside goal_slowdown_dist and can go below v_min if allowed
        v_floor = self.v_min
        if remaining < self.goal_slowdown_dist:
            r = remaining / max(1e-6, self.goal_slowdown_dist)  # 1 → far, 0 → at goal
            ease = r*r*(3.0 - 2.0*r)  # smoothstep
            v_des = v_des * ease
            if self.allow_below_vmin_near_goal:
                v_floor = 0.0

        # Physics-based stopping cap (with a small stop-before-goal margin)
        rem_eff = max(0.0, remaining - self.goal_stop_margin)
        v_goal_cap = math.sqrt(max(0.0, 2.0*self.decel_max*rem_eff))
        v_des = min(v_des, v_goal_cap)

        # Apply floor AFTER goal funnel logic
        v_des = max(v_floor, v_des)

        # Accel limiting
        dv = v_des - self.v_prev
        dv_max = (self.accel_max if dv > 0.0 else self.decel_max) * self.dt
        v_cmd = clamp(self.v_prev + clamp(dv, -dv_max, dv_max), 0.0, 5.0)  # 5.0 is a sanity cap

        # Angular velocity: FF + mild PD
        w_ff = v_cmd * kappa_use
        w_pd = heading_kp * brg_err + heading_kd * d_brg
        w_cmd = clamp(w_ff + w_pd, -self.w_max, self.w_max)

        # Stop at goal (latch + optional final align)
        if self.stop_at_goal and (remaining <= self.goal_pos_tol):
            self.goal_stop_latch = True
            self.goal_dwell_until = self.get_clock().now() + Duration(seconds=self.goal_dwell_s)
            # zero linear; do a quick final align while dwelling (handled at top)
            self._send(0.0, 0.0)
            self.v_prev = 0.0
            return

        # Send
        self._send(v_cmd, w_cmd)
        self.v_prev = v_cmd

        # Debug pubs
        if self.pub_debug:
            self._pub_lookahead(gx, gy)
            self._pub_marker_lookahead(gx, gy)

    # ------------------- cmd_vel helper -------------------
    def _send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuitFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()