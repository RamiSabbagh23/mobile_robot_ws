#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmoothRightFollower (full rewrite)

- Right-wall following with PD control on perpendicular distance and wall angle
- Goal drive mode with LiDAR LOS check (can be disabled)
- Auto-save map once the robot returns within `return_radius` of its start pose:
  * Save as my_mapN.{pgm,yaml} (N increments each run) under:
      - If `map_folder_abs` is set, exactly there.
      - Else under src/<package_name>/<map_folder_rel>/ (preferred) or fallback locations.
  * If Nav2 SaveMap service is enabled (`prefer_nav2_service:=true`) use it
    (same my_mapN prefix). Otherwise write from the last OccupancyGrid.
  * Immediately stop and shut down the node after saving if `shutdown_after_save:=true`.

Run example (explicit path):
  ros2 run autonomous_explorer explorer_node \
    --ros-args \
    -p map_folder_abs:=/home/rami/mobile_robot_ws/src/autonomous_explorer/map \
    -p shutdown_after_save:=true
"""
import os, re, math, datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# Optional: Nav2 SaveMap service
try:
    from nav2_msgs.srv import SaveMap as Nav2SaveMap
except Exception:
    Nav2SaveMap = None

# Optional: locate package share dir
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


# ---------------- utils ----------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def yaw_from_quat(x, y, z, w):
    # ZYX yaw
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


class SmoothRightFollower(Node):
    def __init__(self):
        super().__init__('smooth_right_follower')

        # ---- Topics ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')

        # ---- Frames ----
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')  # could be 'odom' if no global map

        # ---- Goal parameters ----
        self.declare_parameter('enable_goal', False)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_activate_radius', 5.0)
        self.declare_parameter('goal_stop_thresh', 0.1)
        self.declare_parameter('goal_los_width_deg', 8.0)
        self.declare_parameter('goal_clear_margin', 0.30)
        self.declare_parameter('goal_v_max', 2.0)
        self.declare_parameter('goal_w_gain', 1.25)
        self.declare_parameter('goal_slow_radius', 2.0)
        self.declare_parameter('goal_debug', False)

        # ---- Wall / safety distances ----
        self.declare_parameter('desired_right_dist', 2.00)
        self.declare_parameter('right_present_max', 3.20)
        self.declare_parameter('front_stop', 1.60)
        self.declare_parameter('front_clear', 3.20)

        # ---- Base speeds / caps ----
        self.declare_parameter('v_nom', 2.0)
        self.declare_parameter('w_max', 1.75)

        # ---- Control gains (TRACK) ----
        self.declare_parameter('Kp_e', 0.95)
        self.declare_parameter('Kd_e', 0.28)
        self.declare_parameter('Kp_a', 0.85)
        self.declare_parameter('Kd_a', 0.18)

        # ---- Filters & limits ----
        self.declare_parameter('range_ewma_alpha', 0.25)
        self.declare_parameter('deriv_alpha', 0.60)
        self.declare_parameter('deriv_clip', 10.0)
        self.declare_parameter('v_slew', 3.5)
        self.declare_parameter('w_slew', 10.0)
        self.declare_parameter('v_floor', 0.18)

        # ---- Speed shaping ----
        self.declare_parameter('turn_slowdown', 0.65)
        self.declare_parameter('front_slowdown_gain', 1.10)

        # ---- Sensor-aware forward gating ----
        self.declare_parameter('time_headway_s', 0.60)
        self.declare_parameter('sensor_guard_frac', 0.80)
        self.declare_parameter('sensor_guard_margin_m', 0.30)
        self.declare_parameter('reacquire_v_scale', 0.55)

        # ---- Acquisition logic ----
        self.declare_parameter('acquire_consistency_frames', 8)
        self.declare_parameter('acquire_right_max', 2.40)

        # ---- Sector widths (deg) ----
        self.declare_parameter('front_width_deg', 32.0)
        self.declare_parameter('right_width_deg', 42.0)
        self.declare_parameter('fr_width_deg', 30.0)
        self.declare_parameter('left_width_deg', 40.0)

        # ---- Anticipatory left when front narrows ----
        self.declare_parameter('front_turn_start', 3.80)
        self.declare_parameter('front_turn_max',   2.80)
        self.declare_parameter('left_turn_gain',   0.55)
        self.declare_parameter('left_turn_wmax_scale', 0.85)

        # ---- Convex-corner handling ----
        self.declare_parameter('corner_front_thresh', 2.10)
        self.declare_parameter('corner_right_thresh', 1.50)
        self.declare_parameter('corner_min_arc_s',    0.55)
        self.declare_parameter('corner_v_scale',      0.55)
        self.declare_parameter('corner_w_scale',      0.90)
        self.declare_parameter('corner_exit_front',   2.60)
        self.declare_parameter('corner_exit_band',    0.22)
        self.declare_parameter('corner_hysteresis',   0.12)

        # ---- Auto-save map on return-to-start ----
        self.declare_parameter('autosave_on_return', True)
        self.declare_parameter('use_current_pose_as_start', True)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('return_radius', 0.6)
        self.declare_parameter('autosave_min_runtime_s', 20.0)

        # Save destinations and behavior
        self.declare_parameter('package_name', 'autonomous_explorer')
        self.declare_parameter('map_folder_rel', 'map')      # <pkg>/src/<package>/map/
        self.declare_parameter('map_folder_abs', '')         # absolute override (recommended)
        self.declare_parameter('shutdown_after_save', True)

        # Nav2 saver (optional)
        self.declare_parameter('prefer_nav2_service', False)
        self.declare_parameter('map_saver_service', '/map_saver/save_map')
        self.declare_parameter('save_image_format', 'pgm')
        self.declare_parameter('save_free_thresh', 0.196)
        self.declare_parameter('save_occupied_thresh', 0.65)

        # ---- Read params ----
        gp = self.get_parameter
        self.scan_topic      = gp('scan_topic').value
        self.cmd_topic       = gp('cmd_vel_topic').value
        self.map_topic       = gp('map_topic').value

        self.base_frame      = gp('base_frame').value
        self.global_frame    = gp('global_frame').value

        self.enable_goal     = bool(gp('enable_goal').value)
        self.goal_x          = float(gp('goal_x').value)
        self.goal_y          = float(gp('goal_y').value)
        self.goal_radius     = float(gp('goal_activate_radius').value)
        self.goal_thresh     = float(gp('goal_stop_thresh').value)
        self.goal_los_w      = math.radians(float(gp('goal_los_width_deg').value))
        self.goal_clear_m    = float(gp('goal_clear_margin').value)
        self.goal_v_max      = float(gp('goal_v_max').value)
        self.goal_w_gain     = float(gp('goal_w_gain').value)
        self.goal_slow_r     = float(gp('goal_slow_radius').value)
        self.goal_debug      = bool(gp('goal_debug').value)

        self.d_des           = float(gp('desired_right_dist').value)
        self.right_present   = float(gp('right_present_max').value)
        self.front_stop      = float(gp('front_stop').value)
        self.front_clear     = float(gp('front_clear').value)

        self.v_nom           = float(gp('v_nom').value)
        self.w_max           = float(gp('w_max').value)

        self.Kp_e            = float(gp('Kp_e').value)
        self.Kd_e            = float(gp('Kd_e').value)
        self.Kp_a            = float(gp('Kp_a').value)
        self.Kd_a            = float(gp('Kd_a').value)

        self.rng_alpha       = float(gp('range_ewma_alpha').value)
        self.deriv_alpha     = float(gp('deriv_alpha').value)
        self.deriv_clip      = float(gp('deriv_clip').value)
        self.v_slew          = float(gp('v_slew').value)
        self.w_slew          = float(gp('w_slew').value)
        self.v_floor         = float(gp('v_floor').value)

        self.turn_slowdown   = float(gp('turn_slowdown').value)
        self.front_slow_g    = float(gp('front_slowdown_gain').value)

        self.t_headway       = float(gp('time_headway_s').value)
        self.sensor_guard    = float(gp('sensor_guard_frac').value)
        self.sensor_margin   = float(gp('sensor_guard_margin_m').value)
        self.reacq_vscale    = float(gp('reacquire_v_scale').value)

        self.need_frames     = int(gp('acquire_consistency_frames').value)
        self.acquire_max     = float(gp('acquire_right_max').value)

        self.front_w         = math.radians(float(gp('front_width_deg').value))
        self.right_w         = math.radians(float(gp('right_width_deg').value))
        self.fr_w            = math.radians(float(gp('fr_width_deg').value))
        self.left_w          = math.radians(float(gp('left_width_deg').value))

        self.front_turn_start = float(gp('front_turn_start').value)
        self.front_turn_max   = float(gp('front_turn_max').value)
        self.left_turn_gain   = float(gp('left_turn_gain').value)
        self.left_turn_wscale = float(gp('left_turn_wmax_scale').value)

        self.corner_front_thresh = float(gp('corner_front_thresh').value)
        self.corner_right_thresh = float(gp('corner_right_thresh').value)
        self.corner_min_arc_s    = float(gp('corner_min_arc_s').value)
        self.corner_v_scale      = float(gp('corner_v_scale').value)
        self.corner_w_scale      = float(gp('corner_w_scale').value)
        self.corner_exit_front   = float(gp('corner_exit_front').value)
        self.corner_exit_band    = float(gp('corner_exit_band').value)
        self.corner_hys          = float(gp('corner_hysteresis').value)

        # Autosave params
        self.autosave_on_return   = bool(gp('autosave_on_return').value)
        self.use_current_as_start = bool(gp('use_current_pose_as_start').value)
        self.start_x_param        = float(gp('start_x').value)
        self.start_y_param        = float(gp('start_y').value)
        self.return_radius        = float(gp('return_radius').value)
        self.autosave_min_runtime_s = float(gp('autosave_min_runtime_s').value)

        self.package_name         = gp('package_name').value
        self.map_folder_rel       = gp('map_folder_rel').value
        self.map_folder_abs       = gp('map_folder_abs').value
        self.shutdown_after_save  = bool(gp('shutdown_after_save').value)

        self.prefer_nav2_service  = bool(gp('prefer_nav2_service').value)
        self.map_saver_service    = gp('map_saver_service').value
        self.save_image_format    = gp('save_image_format').value
        self.save_free_thresh     = float(gp('save_free_thresh').value)
        self.save_occupied_thresh = float(gp('save_occupied_thresh').value)

        # ---- I/O ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.map_sub  = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 10)
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_topic, 10)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- State ----
        self.state = 'ACQUIRE_RIGHT_WALL'  # TRACK_RIGHT_WALL | CORNER_LEFT | BACKOFF | GOAL_DRIVE | GOAL_STOP
        self.state_enter_t = self.now_s()
        self.acquire_ok_count = 0

        self.last_scan = None
        self.r_right_f = math.inf
        self.r_fr_f    = math.inf
        self.r_front_f = math.inf
        self.r_left_f  = math.inf
        self.rmax_last = math.inf

        self.prev_t = self.now_s()
        self.prev_e = 0.0
        self.prev_alpha = 0.0
        self.de_f = 0.0
        self.da_f = 0.0

        self.v_cmd_prev = 0.0
        self.w_cmd_prev = 0.0

        self.debug_t_last = 0.0
        self.goal_dbg_last = 0.0

        self.start_pose = None
        self.autosave_done = False
        self.start_time = self.now_s()

        # Nav2 SaveMap client
        self.save_client = None
        if Nav2SaveMap is not None and self.prefer_nav2_service:
            try:
                self.save_client = self.create_client(Nav2SaveMap, self.map_saver_service)
            except Exception:
                self.save_client = None

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info(
            f'Started SmoothRightFollower  scan="{self.scan_topic}"  cmd="{self.cmd_topic}"  '
            f'goal=({self.goal_x:.2f},{self.goal_y:.2f})  enable_goal={self.enable_goal}'
        )

        self.add_on_set_parameters_callback(self._on_param_change)

    # ---------- Time / state ----------
    def now_s(self): 
        return self.get_clock().now().nanoseconds * 1e-9

    def t_in_state(self): 
        return self.now_s() - self.state_enter_t

    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.state_enter_t = self.now_s()

    def send(self, v, w):
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    # ---------- Subscribers ----------
    def on_map(self, msg: OccupancyGrid):
        self.last_map = msg

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        if msg and msg.range_max > 0.0:
            self.rmax_last = msg.range_max

    # ---------- LiDAR helpers ----------
    def sector_min(self, center, width):
        msg = self.last_scan
        if msg is None or msg.angle_increment == 0.0:
            return math.inf
        a0 = msg.angle_min
        aincr = msg.angle_increment
        n = len(msg.ranges)
        span = n * aincr
        start = center - width / 2.0
        stop  = center + width / 2.0

        def idx_of(theta):
            off = (theta - a0) % span
            return int(clamp(math.floor(off / aincr), 0, n - 1))

        i_lo = idx_of(start); i_hi = idx_of(stop)

        def norm_rng(r):
            rmin = max(0.0, msg.range_min)
            rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
            if r is None or not math.isfinite(r) or r <= rmin or r > rmax:
                return float('inf')
            return r

        if i_hi >= i_lo:
            vals = [norm_rng(r) for r in msg.ranges[i_lo:i_hi + 1]]
        else:
            vals = [norm_rng(r) for r in (msg.ranges[i_lo:] + msg.ranges[:i_hi + 1])]
        return min(vals) if vals else math.inf

    def wall_alpha_and_D(self, r_right, r_fr):
        dtheta = math.pi / 4.0  # 45°
        if not math.isfinite(r_right) or not math.isfinite(r_fr):
            return 0.0, math.inf
        denom = r_right * math.sin(dtheta)
        if abs(denom) < 1e-6:
            return 0.0, math.inf
        alpha = math.atan((r_right * math.cos(dtheta) - r_fr) / denom)
        D = r_fr * math.cos(alpha)
        if D < 0.0:
            D = math.inf
        return alpha, D

    # ---------- Pose / goal ----------
    def _robot_pose_global(self):
        try:
            ts = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
            t = ts.transform.translation
            q = ts.transform.rotation
            x, y = float(t.x), float(t.y)
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _goal_vector(self):
        if not self.enable_goal:
            return None
        pose = self._robot_pose_global()
        if pose is None:
            return None
        rx, ry, ryaw = pose
        dx = self.goal_x - rx
        dy = self.goal_y - ry
        dist = math.hypot(dx, dy)
        brg = math.atan2(dy, dx) - ryaw
        brg = math.atan2(math.sin(brg), math.cos(brg))
        if self.last_scan is None:
            occluded = True
        else:
            r_min = self.sector_min(brg, self.goal_los_w)
            occluded = (r_min + self.goal_clear_m) < dist
        return dist, brg, occluded

    def _maybe_record_start_pose(self):
        if self.start_pose is not None:
            return
        if not self.use_current_as_start:
            self.start_pose = (self.start_x_param, self.start_y_param)
            self.get_logger().info(f"[autosave] Using configured start pose: {self.start_pose}")
            return
        pose = self._robot_pose_global()
        if pose is None:
            return
        x, y, _ = pose
        self.start_pose = (x, y)
        self.get_logger().info(f"[autosave] Recorded start pose from TF: ({x:.2f}, {y:.2f})")

    # ---------- Control loop ----------
    def loop(self):
        now = self.now_s()
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        if self.autosave_on_return:
            self._maybe_record_start_pose()

        if self.last_scan is None:
            self._send_smooth(0.0, 0.3, dt)  # slow spin to get data
            self._maybe_autosave(now)
            return

        # Sector minima (EWMA)
        r_front = self.sector_min(0.0, self.front_w)
        r_right = self.sector_min(-math.pi/2, self.right_w)
        r_fr    = self.sector_min(-math.pi/4, self.fr_w)
        r_left  = self.sector_min(+math.pi/2, self.left_w)

        a = clamp(self.rng_alpha, 0.0, 1.0)
        def ewma(prev, new): return (1.0 - a) * prev + a * new
        self.r_front_f = ewma(self.r_front_f, r_front) if math.isfinite(self.r_front_f) else r_front
        self.r_right_f = ewma(self.r_right_f, r_right) if math.isfinite(self.r_right_f) else r_right
        self.r_fr_f    = ewma(self.r_fr_f,    r_fr)    if math.isfinite(self.r_fr_f)    else r_fr
        self.r_left_f  = ewma(self.r_left_f,  r_left)  if math.isfinite(self.r_left_f)  else r_left

        alpha, D = self.wall_alpha_and_D(self.r_right_f, self.r_fr_f)

        # Debug
        if now - self.debug_t_last > 0.6:
            self.get_logger().info(
                f"state={self.state}  F={self.r_front_f:.2f} R={self.r_right_f:.2f} L={self.r_left_f:.2f} "
                f"FR={self.r_fr_f:.2f}  α={alpha:.3f} D={(D if math.isfinite(D) else float('inf')):.2f}"
            )
            self.debug_t_last = now

        # Goal vector
        gv = self._goal_vector() if self.enable_goal else None
        if gv is not None:
            gdist, gbrg, goccl = gv
            if self.goal_debug and (now - self.goal_dbg_last) > 0.8:
                self.get_logger().info(f"GOAL: d={gdist:.2f} brg={gbrg:+.2f} occl={goccl}")
                self.goal_dbg_last = now
        else:
            gdist, gbrg, goccl = math.inf, 0.0, True

        # ------------- State machine -------------
        # Dead-end detection
        if (self.r_front_f < self.front_stop and
            self.r_right_f < self.corner_right_thresh - self.corner_hys and
            self.r_left_f  < self.corner_right_thresh - self.corner_hys and
            self.state not in ('BACKOFF', 'GOAL_STOP')):
            self.set_state('BACKOFF')

        # Hard front safety
        if (self.r_front_f < self.front_stop and self.state not in ('BACKOFF', 'CORNER_LEFT', 'GOAL_STOP')):
            if self._is_corner(self.r_front_f, self.r_right_f) and self.state != 'GOAL_DRIVE':
                self._enter_corner()
            else:
                self.set_state('BACKOFF')

        # Enter goal drive if close + clear
        if self.state in ('ACQUIRE_RIGHT_WALL', 'TRACK_RIGHT_WALL', 'CORNER_LEFT') and self.enable_goal:
            if gdist <= self.goal_radius and not goccl:
                self.set_state('GOAL_DRIVE')

        if self.state == 'ACQUIRE_RIGHT_WALL':
            if self._is_corner(self.r_front_f, self.r_right_f):
                self._enter_corner()
            else:
                present = math.isfinite(D) and (D < self.acquire_max)
                self.acquire_ok_count = self.acquire_ok_count + 1 if present else 0

                w_seek = -0.25 * self.w_max
                v_seek = 0.55 * self.v_nom
                v_seek = self._forward_speed_gate(v_seek, self.r_front_f, D)

                if self.acquire_ok_count >= self.need_frames:
                    self.set_state('TRACK_RIGHT_WALL')
                    self.prev_e = 0.0; self.prev_alpha = alpha; self.de_f = 0.0; self.da_f = 0.0

                self._send_smooth(v_seek, w_seek, dt)

        elif self.state == 'TRACK_RIGHT_WALL':
            if self._is_corner(self.r_front_f, self.r_right_f):
                self._enter_corner()
            else:
                e = self.d_des - (D if math.isfinite(D) else self.d_des)
                de = (e - self.prev_e) / dt
                da = (alpha - self.prev_alpha) / dt
                self.prev_e = e; self.prev_alpha = alpha

                b = clamp(self.deriv_alpha, 0.0, 1.0)
                self.de_f = (1.0 - b) * self.de_f + b * clamp(de, -self.deriv_clip, +self.deriv_clip)
                self.da_f = (1.0 - b) * self.da_f + b * clamp(da, -self.deriv_clip, +self.deriv_clip)

                w_cmd = (self.Kp_e * e) + (self.Kd_e * self.de_f) + (self.Kp_a * alpha) + (self.Kd_a * self.da_f)

                # anticipatory left
                fts, ftm, f = self.front_turn_start, self.front_turn_max, self.r_front_f
                if f < fts:
                    if f <= ftm: wboost = 1.0
                    else:
                        x = (fts - f) / max(1e-6, (fts - ftm))
                        x = clamp(x, 0.0, 1.0)
                        wboost = (3*x*x - 2*x*x*x)
                    w_cmd += wboost * self.left_turn_gain * (self.left_turn_wscale * self.w_max)

                w_cmd = clamp(w_cmd, -self.w_max, +self.w_max)

                turn_frac = min(1.0, abs(w_cmd) / max(1e-6, self.w_max))
                v_cmd = self.v_nom * (1.0 - self.turn_slowdown * turn_frac)
                v_cmd = self._forward_speed_gate(v_cmd, self.r_front_f, D)

                if not math.isfinite(D) or D > self.right_present:
                    w_cmd = clamp(w_cmd - 0.22 * self.w_max, -self.w_max, +self.w_max)

                self._send_smooth(v_cmd, w_cmd, dt)

        elif self.state == 'CORNER_LEFT':
            t = self.t_in_state()
            v_arc = self.v_nom * self.corner_v_scale
            w_arc = + self.w_max * self.corner_w_scale

            alpha, D = self.wall_alpha_and_D(self.r_right_f, self.r_fr_f)
            front_ok = self.r_front_f > self.corner_exit_front
            right_ok = math.isfinite(D) and (abs(D - self.d_des) < self.corner_exit_band)

            if t >= self.corner_min_arc_s and (front_ok or right_ok):
                self.set_state('TRACK_RIGHT_WALL')
                self.prev_e = 0.0; self.prev_alpha = alpha; self.de_f = 0.0; self.da_f = 0.0

            self._send_smooth(v_arc, w_arc, dt)

        elif self.state == 'BACKOFF':
            if self.t_in_state() < 0.55:
                self._send_smooth(-0.18, 0.0, dt)
            else:
                self._send_smooth(0.0, 0.0, dt)
                self.acquire_ok_count = 0
                self.set_state('ACQUIRE_RIGHT_WALL')

        elif self.state == 'GOAL_DRIVE':
            if goccl or (gdist > self.goal_radius + 0.5):
                self.set_state('ACQUIRE_RIGHT_WALL')
                self._send_smooth(0.0, 0.0, dt)
                self._maybe_autosave(now)
                return

            if gdist <= self.goal_thresh:
                self.set_state('GOAL_STOP')
                self._send_smooth(0.0, 0.0, dt)
                self._maybe_autosave(now)
                return

            w_des = clamp(self.goal_w_gain * gbrg, -self.w_max, +self.w_max)
            v_des = min(self.goal_v_max, self.v_nom)
            if gdist < self.goal_slow_r:
                v_des *= (gdist / max(1e-6, self.goal_slow_r))
            turn_frac = min(1.0, abs(w_des) / max(1e-6, self.w_max))
            v_des *= (1.0 - self.turn_slowdown * turn_frac)
            v_des = self._forward_speed_gate_goal(v_des, self.r_front_f)

            self._send_smooth(v_des, w_des, dt)

        elif self.state == 'GOAL_STOP':
            self._send_smooth(0.0, 0.0, dt)

        # Autosave check each loop
        self._maybe_autosave(now)

    # ---------- Helpers ----------
    def _is_corner(self, front_d, right_d):
        return (front_d < self.corner_front_thresh) and (right_d < self.corner_right_thresh)

    def _enter_corner(self):
        if self.state != 'GOAL_DRIVE':
            self.set_state('CORNER_LEFT')

    def _forward_speed_gate(self, v, front_dist, D):
        if front_dist <= self.front_stop:
            v = 0.0
        elif front_dist < self.front_clear:
            x = (front_dist - self.front_stop) / max(1e-6, (self.front_clear - self.front_stop))
            v *= (x ** self.front_slow_g)
        v = min(v, max(0.0, (front_dist - self.front_stop) / max(1e-6, self.t_headway)))
        usable = max(0.0, self.sensor_guard * self.rmax_last - self.sensor_margin)
        v = min(v, max(0.0, (usable - self.front_stop) / max(1e-6, self.t_headway)))
        if (not math.isfinite(D)) or (D > self.right_present):
            v *= self.reacq_vscale
        return clamp(v, 0.0, self.v_nom)

    def _forward_speed_gate_goal(self, v, front_dist):
        if front_dist <= self.front_stop:
            v = 0.0
        elif front_dist < self.front_clear:
            x = (front_dist - self.front_stop) / max(1e-6, (self.front_clear - self.front_stop))
            v *= (x ** self.front_slow_g)
        v = min(v, max(0.0, (front_dist - self.front_stop) / max(1e-6, self.t_headway)))
        usable = max(0.0, self.sensor_guard * self.rmax_last - self.sensor_margin)
        v = min(v, max(0.0, (usable - self.front_stop) / max(1e-6, self.t_headway)))
        return clamp(v, 0.0, min(self.v_nom, self.goal_v_max))

    def _send_smooth(self, v_des, w_des, dt):
        w_des = clamp(w_des, -self.w_max, +self.w_max)
        v_des = clamp(v_des, 0.0 if v_des >= 0.0 else -self.v_floor, self.v_nom)

        dv_max = self.v_slew * dt
        v_cmd = self.v_cmd_prev + clamp(v_des - self.v_cmd_prev, -dv_max, +dv_max)
        if v_des > 0.05:
            if v_cmd > 0.0:
                v_cmd = max(self.v_floor, v_cmd)
        else:
            if abs(v_cmd) < 0.02:
                v_cmd = 0.0
        dw_max = self.w_slew * dt
        w_cmd = self.w_cmd_prev + clamp(w_des - self.w_cmd_prev, -dw_max, +dw_max)
        self.v_cmd_prev = v_cmd
        self.w_cmd_prev = w_cmd
        self.send(v_cmd, w_cmd)

    # ---------- Auto-save ----------
    def _maybe_autosave(self, now_s: float):
        if not self.autosave_on_return or self.autosave_done or self.start_pose is None:
            return
        if (now_s - self.start_time) < self.autosave_min_runtime_s:
            return
        pose = self._robot_pose_global()
        if pose is None:
            return
        x, y, _ = pose
        sx, sy = self.start_pose
        dist = math.hypot(x - sx, y - sy)
        if dist > self.return_radius:
            return

        # Near start: stop and save
        self.get_logger().info(f"[autosave] Within {self.return_radius:.2f} m of start (d={dist:.2f}). Stopping and saving map...")
        self._send_smooth(0.0, 0.0, 1.0)
        ok, path = self._do_save_map()
        if ok:
            self.get_logger().info(f"[autosave] Map saved to: {path}")
        else:
            self.get_logger().error("[autosave] Map save FAILED.")
        self.autosave_done = True
        self.set_state('GOAL_STOP')
        self._send_smooth(0.0, 0.0, 1.0)
        if self.shutdown_after_save:
            self._request_shutdown()

    def _resolve_maps_dir(self):
        """Prefer src/<package_name>/<map_folder_rel>/.
           If map_folder_abs is set, use it. Else:
           - Try deriving workspace root from share dir: <ws>/install/<pkg>/share/<pkg> ⟶ <ws>/src/<pkg>/<rel>
           - Fallback to share/<pkg>/<rel>
           - Fallback to CWD/<rel>
        """
        # Absolute override
        p = str(self.map_folder_abs).strip()
        if p:
            if not os.path.isabs(p):
                p = os.path.abspath(os.path.join(os.getcwd(), p))
            os.makedirs(p, exist_ok=True)
            return p

        share_dir = None
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory(self.package_name)
            except Exception:
                share_dir = None

        if share_dir is not None:
            # share_dir: <ws>/install/<pkg>/share/<pkg>
            install_pkg = os.path.dirname(os.path.dirname(share_dir))   # .../install/<pkg>
            install_root = os.path.dirname(install_pkg)                 # .../install
            ws_root = os.path.dirname(install_root)                     # .../<ws>
            candidate_src = os.path.join(ws_root, 'src', self.package_name)
            if os.path.isdir(candidate_src):
                maps_dir = os.path.join(candidate_src, self.map_folder_rel)
                os.makedirs(maps_dir, exist_ok=True)
                return maps_dir
            # fallback to share
            maps_dir = os.path.join(share_dir, self.map_folder_rel)
            os.makedirs(maps_dir, exist_ok=True)
            return maps_dir

        # Last resort: CWD
        base_dir = os.getcwd()
        self.get_logger().warn(f"[autosave] Could not resolve share/src paths. Saving under CWD: {base_dir}")
        maps_dir = os.path.join(base_dir, self.map_folder_rel)
        os.makedirs(maps_dir, exist_ok=True)
        return maps_dir

    # ---- NEW: choose next my_mapN prefix in maps_dir ----
    def _next_my_map_prefix(self, maps_dir: str) -> str:
        """
        Scan maps_dir for files named my_map<number>.(yaml|pgm|png)
        and pick the next available number.
        """
        base = "my_map"
        max_n = 0
        try:
            for fn in os.listdir(maps_dir):
                m = re.match(rf'^{re.escape(base)}(\d+)\.(yaml|pgm|png)$', fn)
                if m:
                    n = int(m.group(1))
                    if n > max_n:
                        max_n = n
        except Exception:
            pass
        next_n = max_n + 1
        return os.path.join(maps_dir, f"{base}{next_n}")

    def _do_save_map(self):
        """Try Nav2 SaveMap service, else write from last OccupancyGrid. Returns (ok, yaml_path_or_prefix)."""
        maps_dir = self._resolve_maps_dir()
        # Use incremental my_mapN prefix instead of timestamp
        path_prefix = self._next_my_map_prefix(maps_dir)

        # Nav2 service
        if self.prefer_nav2_service and self.save_client is not None and Nav2SaveMap is not None:
            if not self.save_client.wait_for_service(timeout_sec=1.5):
                self.get_logger().warn(f"[autosave] Service {self.map_saver_service} unavailable; falling back to local writer.")
            else:
                try:
                    req = Nav2SaveMap.Request()
                    req.map_topic = self.map_topic
                    req.map_url = path_prefix
                    req.image_format = self.save_image_format
                    req.map_mode = 'trinary'
                    req.free_thresh = float(self.save_free_thresh)
                    req.occupied_thresh = float(self.save_occupied_thresh)
                    fut = self.save_client.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
                    if fut.done() and fut.result() and getattr(fut.result(), 'success', False):
                        return True, path_prefix + ".yaml"
                    else:
                        self.get_logger().warn("[autosave] Nav2 SaveMap returned failure; falling back to local writer.")
                except Exception as e:
                    self.get_logger().warn(f"[autosave] Nav2 SaveMap call error: {e}. Falling back to local writer.")

        # Fallback: write PGM+YAML from OccupancyGrid
        if getattr(self, 'last_map', None) is None:
            self.get_logger().error("[autosave] No OccupancyGrid received on map topic; cannot save locally.")
            return False, path_prefix

        ok = self._write_map_files_from_occupancy_grid(self.last_map, path_prefix)
        return ok, (path_prefix + ".yaml" if ok else path_prefix)

    def _write_map_files_from_occupancy_grid(self, msg: OccupancyGrid, path_prefix: str) -> bool:
        try:
            width  = msg.info.width
            height = msg.info.height
            res    = msg.info.resolution
            ox     = msg.info.origin.position.x
            oy     = msg.info.origin.position.y
            q      = msg.info.origin.orientation
            oyaw   = yaw_from_quat(q.x, q.y, q.z, q.w)

            data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
            img = np.zeros((height, width), dtype=np.uint8)
            unknown_mask = (data < 0)
            known = np.clip(data, 0, 100)
            img_known = (255 - (known * 255 // 100)).astype(np.uint8)
            img[:, :] = 205
            img[~unknown_mask] = img_known[~unknown_mask]
            img[img == 255] = 254
            img = np.flipud(img)  # flip vertically to match ROS map convention

            # Write PGM
            img_path = path_prefix + ".pgm"
            with open(img_path, 'wb') as f:
                header = f"P5\n# CREATOR: SmoothRightFollower autosave\n{width} {height}\n255\n"
                f.write(header.encode('ascii'))
                f.write(img.tobytes(order='C'))

            # Write YAML
            yaml_path = path_prefix + ".yaml"
            yaml_text = (
                f"image: {os.path.basename(img_path)}\n"
                f"resolution: {res:.9f}\n"
                f"origin: [{ox:.9f}, {oy:.9f}, {oyaw:.9f}]\n"
                f"negate: 0\n"
                f"occupied_thresh: {self.save_occupied_thresh:.3f}\n"
                f"free_thresh: {self.save_free_thresh:.3f}\n"
            )
            with open(yaml_path, 'w') as f:
                f.write(yaml_text)
            return True
        except Exception as e:
            self.get_logger().error(f"[autosave] Error writing map files: {e}")
            return False

    # ---------- Shutdown ----------
    def _request_shutdown(self):
        if getattr(self, '_shutdown_requested', False):
            return
        self._shutdown_requested = True
        self.get_logger().info("[autosave] Shutting down node...")
        def _do():
            try:
                t.cancel()
            except Exception:
                pass
            try:
                self.send(0.0, 0.0)
            except Exception:
                pass
            try:
                self.destroy_node()
            finally:
                rclpy.shutdown()
        t = self.create_timer(0.25, _do)  # one-shot

    # ---------- Live param updates ----------
    def _on_param_change(self, params):
        for p in params:
            name = p.name
            if name in ('Kp_e','Kd_e','Kp_a','Kd_a','v_nom','w_max','goal_x','goal_y','goal_v_max','goal_w_gain','goal_slow_radius'):
                setattr(self, name if name != 'goal_slow_radius' else 'goal_slow_r', float(p.value))
            elif name in ('enable_goal','goal_debug','autosave_on_return','use_current_pose_as_start',
                          'prefer_nav2_service','shutdown_after_save'):
                setattr(self, name, bool(p.value))
            elif name in ('start_x','start_y','return_radius','autosave_min_runtime_s',
                          'save_free_thresh','save_occupied_thresh'):
                if name == 'start_x': self.start_x_param = float(p.value)
                elif name == 'start_y': self.start_y_param = float(p.value)
                elif name == 'return_radius': self.return_radius = float(p.value)
                elif name == 'autosave_min_runtime_s': self.autosave_min_runtime_s = float(p.value)
                elif name == 'save_free_thresh': self.save_free_thresh = float(p.value)
                elif name == 'save_occupied_thresh': self.save_occupied_thresh = float(p.value)
            elif name in ('map_topic','map_saver_service','package_name','map_folder_rel','save_image_format',
                          'base_frame','global_frame','cmd_vel_topic','scan_topic','map_folder_abs'):
                if name == 'cmd_vel_topic': self.cmd_topic = str(p.value)
                elif name == 'scan_topic': self.scan_topic = str(p.value)
                elif name == 'map_folder_abs': self.map_folder_abs = str(p.value)
                else: setattr(self, name, str(p.value))
            elif name == 'desired_right_dist':
                self.d_des = float(p.value)
            elif name in (
                'range_ewma_alpha','deriv_alpha','deriv_clip','v_slew','w_slew','v_floor',
                'turn_slowdown','front_slowdown_gain','front_stop','front_clear',
                'right_present_max','acquire_consistency_frames','acquire_right_max',
                'front_turn_start','front_turn_max','left_turn_gain','left_turn_wmax_scale',
                'corner_front_thresh','corner_right_thresh','corner_min_arc_s',
                'corner_v_scale','corner_w_scale','corner_exit_front','corner_exit_band','corner_hysteresis',
                'front_width_deg','right_width_deg','fr_width_deg','left_width_deg',
                'time_headway_s','sensor_guard_frac','sensor_guard_margin_m','reacquire_v_scale',
                'goal_activate_radius','goal_stop_thresh','goal_los_width_deg','goal_clear_margin'
            ):
                if name == 'acquire_consistency_frames':
                    self.need_frames = int(p.value)
                elif name in ('front_width_deg','right_width_deg','fr_width_deg','left_width_deg'):
                    val = float(p.value)
                    if name == 'front_width_deg': self.front_w = math.radians(val)
                    if name == 'right_width_deg': self.right_w = math.radians(val)
                    if name == 'fr_width_deg':    self.fr_w    = math.radians(val)
                    if name == 'left_width_deg':  self.left_w  = math.radians(val)
                elif name == 'goal_los_width_deg':
                    self.goal_los_w = math.radians(float(p.value))
                else:
                    mapname = {
                        'range_ewma_alpha':'rng_alpha',
                        'deriv_alpha':'deriv_alpha',
                        'deriv_clip':'deriv_clip',
                        'v_slew':'v_slew',
                        'w_slew':'w_slew',
                        'v_floor':'v_floor',
                        'turn_slowdown':'turn_slowdown',
                        'front_slowdown_gain':'front_slow_g',
                        'front_stop':'front_stop',
                        'front_clear':'front_clear',
                        'right_present_max':'right_present',
                        'acquire_right_max':'acquire_max',
                        'front_turn_start':'front_turn_start',
                        'front_turn_max':'front_turn_max',
                        'left_turn_gain':'left_turn_gain',
                        'left_turn_wmax_scale':'left_turn_wscale',
                        'corner_front_thresh':'corner_front_thresh',
                        'corner_right_thresh':'corner_right_thresh',
                        'corner_min_arc_s':'corner_min_arc_s',
                        'corner_v_scale':'corner_v_scale',
                        'corner_w_scale':'corner_w_scale',
                        'corner_exit_front':'corner_exit_front',
                        'corner_exit_band':'corner_exit_band',
                        'corner_hysteresis':'corner_hys',
                        'time_headway_s':'t_headway',
                        'sensor_guard_frac':'sensor_guard',
                        'sensor_guard_margin_m':'sensor_margin',
                        'reacquire_v_scale':'reacq_vscale',
                        'goal_activate_radius':'goal_radius',
                        'goal_stop_thresh':'goal_thresh',
                        'goal_clear_margin':'goal_clear_m',
                    }
                    attr = mapname.get(name, None)
                    if attr is not None:
                        setattr(self, attr, float(p.value))
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SmoothRightFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
