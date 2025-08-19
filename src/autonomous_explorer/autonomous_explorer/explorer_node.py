#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

def clamp(x, lo, hi): return max(lo, min(hi, x))

class SmoothRightFollower(Node):
    """
    High-speed right-wall follower with larger clearances and robust corner handling.

    States:
      - ACQUIRE_RIGHT_WALL: forward with slight right bias until a consistent right wall is found.
      - TRACK_RIGHT_WALL: PD on right offset and wall angle (beams −90°, −45°), plus anticipatory
                          left bend as the front narrows. Forward speed is limited by front distance,
                          sensor detection headroom, turn demand, and right-wall confidence.
      - CORNER_LEFT: when front & right are both close (convex inside corner), commit to a left arc
                     for a minimum time (ignores PD to avoid stall), then resume tracking.
      - BACKOFF: only for tight dead-ends (front+right+left close).

    Two-beam geometry (−90° ≡ right, −45° ≡ front-right):
        dθ = 45° = π/4
        r1 = range(-90°), r2 = range(-45°)
        α  = atan((r1*cos dθ - r2) / (r1*sin dθ))
        D  = r2 * cos(α)     # perpendicular distance to right wall
    """

    def __init__(self):
        super().__init__('smooth_right_follower')

        # ---- Topics ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---- Stand-off & detection (increased distances) ----
        self.declare_parameter('desired_right_dist', 2.00)   # larger stand-off to right wall (m)
        self.declare_parameter('right_present_max', 3.20)    # consider wall present up to this (m)

        # ---- Front safety / clearance (increased) ----
        self.declare_parameter('front_stop', 1.60)           # HARD stop distance (m)
        self.declare_parameter('front_clear', 3.20)          # full-speed above this (m)

        # ---- Base speeds / caps (tuned for ~3 m/s) ----
        self.declare_parameter('v_nom', 2.0)                 # nominal linear speed (m/s)
        self.declare_parameter('w_max', 1.75)                # yaw rate cap (rad/s)

        # ---- Control gains (TRACK) ----
        self.declare_parameter('Kp_e', 0.95)                 # dist error P
        self.declare_parameter('Kd_e', 0.28)                 # dist error D
        self.declare_parameter('Kp_a', 0.85)                 # angle P
        self.declare_parameter('Kd_a', 0.18)                 # angle D

        # ---- Filters & limits ----
        self.declare_parameter('range_ewma_alpha', 0.25)     # EWMA for sector minima
        self.declare_parameter('deriv_alpha', 0.60)          # EWMA for derivatives
        self.declare_parameter('deriv_clip', 10.0)           # clamp on |d/dt|
        self.declare_parameter('v_slew', 3.5)                # m/s^2  linear slew
        self.declare_parameter('w_slew', 10.0)               # rad/s^2 yaw slew
        self.declare_parameter('v_floor', 0.18)              # min forward speed when moving

        # ---- Speed shaping ----
        self.declare_parameter('turn_slowdown', 0.65)        # reduce v with |w|
        self.declare_parameter('front_slowdown_gain', 1.10)  # exponent in front gating

        # ---- Sensor-aware forward gating (NEW) ----
        # Limit speed by required detection headroom from LiDAR too.
        self.declare_parameter('time_headway_s', 0.60)       # desired headway time (s)
        self.declare_parameter('sensor_guard_frac', 0.80)    # use only this fraction of range_max
        self.declare_parameter('sensor_guard_margin_m', 0.30)# subtract small margin (m)
        self.declare_parameter('reacquire_v_scale', 0.55)    # v scale while wall confidence is low

        # ---- Acquisition logic ----
        self.declare_parameter('acquire_consistency_frames', 8)
        self.declare_parameter('acquire_right_max', 2.40)    # > d_des but not huge

        # ---- Sector widths (deg) ----
        self.declare_parameter('front_width_deg', 32.0)
        self.declare_parameter('right_width_deg', 42.0)
        self.declare_parameter('fr_width_deg', 30.0)
        self.declare_parameter('left_width_deg', 40.0)

        # ---- Anticipatory left when front narrows (TRACK) ----
        self.declare_parameter('front_turn_start', 3.80)     # start bending left early
        self.declare_parameter('front_turn_max',   2.80)     # strongest boost below this
        self.declare_parameter('left_turn_gain',   0.55)     # gentle; still capped by w_max
        self.declare_parameter('left_turn_wmax_scale', 0.85) # fraction of w_max for the boost

        # ---- Convex-corner handling (decisive left arc) ----
        self.declare_parameter('corner_front_thresh', 2.10)  # enter CORNER_LEFT earlier
        self.declare_parameter('corner_right_thresh', 1.50)  # treat right as close (~0.75*d_des)
        self.declare_parameter('corner_min_arc_s',    0.55)  # commit duration
        self.declare_parameter('corner_v_scale',      0.55)  # v_nom factor during arc
        self.declare_parameter('corner_w_scale',      0.90)  # w_max factor during arc
        self.declare_parameter('corner_exit_front',   2.60)  # exit if front > this
        self.declare_parameter('corner_exit_band',    0.22)  # or |D - d_des| < this
        self.declare_parameter('corner_hysteresis',   0.12)

        # ---- Read params ----
        gp = self.get_parameter
        self.scan_topic      = gp('scan_topic').value
        self.cmd_topic       = gp('cmd_vel_topic').value

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

        # ---- I/O ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_topic, 10)

        # ---- State ----
        self.state = 'ACQUIRE_RIGHT_WALL'   # TRACK_RIGHT_WALL | CORNER_LEFT | BACKOFF
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

        # 20 Hz control (3 m/s ≈ 0.15 m per tick)
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(
            f'Started SmoothRightFollower  scan="{self.scan_topic}"  cmd="{self.cmd_topic}"  v_nom={self.v_nom:.2f}'
        )

        self.add_on_set_parameters_callback(self._on_param_change)

    # ---------- Utility ----------
    def now_s(self): return self.get_clock().now().nanoseconds * 1e-9
    def t_in_state(self): return self.now_s() - self.state_enter_t
    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.state_enter_t = self.now_s()

    def send(self, v, w):
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    # ---------- Laser helpers ----------
    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        if msg and msg.range_max > 0.0:
            self.rmax_last = msg.range_max

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

    # ---------- Control loop ----------
    def loop(self):
        now = self.now_s()
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        if self.last_scan is None:
            self._send_smooth(0.0, 0.3, dt)  # slow spin until we have data
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

        # Periodic debug
        if now - self.debug_t_last > 0.6:
            self.get_logger().info(
                f"state={self.state}  F={self.r_front_f:.2f} R={self.r_right_f:.2f} L={self.r_left_f:.2f} "
                f"FR={self.r_fr_f:.2f}  α={alpha:.3f} D={(D if math.isfinite(D) else float('inf')):.2f}"
            )
            self.debug_t_last = now

        # Dead-end (tight box) → BACKOFF
        if (self.r_front_f < self.front_stop and
            self.r_right_f < self.corner_right_thresh - self.corner_hys and
            self.r_left_f  < self.corner_right_thresh - self.corner_hys and
            self.state != 'BACKOFF'):
            self.set_state('BACKOFF')

        # Critical front safety
        if self.r_front_f < self.front_stop and self.state not in ('BACKOFF', 'CORNER_LEFT'):
            if self._is_corner(self.r_front_f, self.r_right_f):
                self._enter_corner()
            else:
                self.set_state('BACKOFF')

        # --- State machine ---
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
                # Errors & derivatives
                e = self.d_des - (D if math.isfinite(D) else self.d_des)
                de = (e - self.prev_e) / dt
                da = (alpha - self.prev_alpha) / dt
                self.prev_e = e; self.prev_alpha = alpha

                b = clamp(self.deriv_alpha, 0.0, 1.0)
                self.de_f = (1.0 - b) * self.de_f + b * clamp(de, -self.deriv_clip, +self.deriv_clip)
                self.da_f = (1.0 - b) * self.da_f + b * clamp(da, -self.deriv_clip, +self.deriv_clip)

                # PD yaw
                w_cmd = (self.Kp_e * e) + (self.Kd_e * self.de_f) + (self.Kp_a * alpha) + (self.Kd_a * self.da_f)

                # Anticipatory left boost as front narrows
                fts, ftm, f = self.front_turn_start, self.front_turn_max, self.r_front_f
                if f < fts:
                    if f <= ftm: w = 1.0
                    else:
                        x = (fts - f) / max(1e-6, (fts - ftm)); x = clamp(x, 0.0, 1.0)
                        w = (3*x*x - 2*x*x*x)
                    w_cmd += w * self.left_turn_gain * (self.left_turn_wscale * self.w_max)

                w_cmd = clamp(w_cmd, -self.w_max, +self.w_max)

                # Base forward speed shaped by turn demand
                turn_frac = min(1.0, abs(w_cmd) / max(1e-6, self.w_max))
                v_cmd = self.v_nom * (1.0 - self.turn_slowdown * turn_frac)

                # Sensor-aware forward gating (front + sensor range + wall confidence)
                v_cmd = self._forward_speed_gate(v_cmd, self.r_front_f, D)

                # If wall lost, bias right to re-acquire
                if not math.isfinite(D) or D > self.right_present:
                    w_cmd = clamp(w_cmd - 0.22 * self.w_max, -self.w_max, +self.w_max)

                self._send_smooth(v_cmd, w_cmd, dt)

        elif self.state == 'CORNER_LEFT':
            t = self.t_in_state()
            v_arc = self.v_nom * self.corner_v_scale
            w_arc = + self.w_max * self.corner_w_scale  # left = +

            # Exit after dwell if front clear OR right offset back in band
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

    # ---------- Helpers ----------
    def _is_corner(self, front_d, right_d):
        return (front_d < self.corner_front_thresh) and (right_d < self.corner_right_thresh)

    def _enter_corner(self):
        self.set_state('CORNER_LEFT')

    def _forward_speed_gate(self, v, front_dist, D):
        """
        Gate forward speed by:
          1) Front clearance (smooth exponent gate)
          2) Time-headway rule (v <= (front - stop)/t_headway)
          3) Sensor detection headroom: only a fraction of LiDAR range is considered usable
          4) Right-wall confidence: slow down while wall is not confidently tracked
        """
        # (1) Smooth front gate
        if front_dist <= self.front_stop:
            v = 0.0
        elif front_dist < self.front_clear:
            x = (front_dist - self.front_stop) / max(1e-6, (self.front_clear - self.front_stop))
            v *= (x ** self.front_slow_g)

        # (2) Time headway gate
        v = min(v, max(0.0, (front_dist - self.front_stop) / max(1e-6, self.t_headway)))

        # (3) Sensor detection headroom (use only a safe fraction of range_max minus a margin)
        usable = max(0.0, self.sensor_guard * self.rmax_last - self.sensor_margin)
        v = min(v, max(0.0, (usable - self.front_stop) / max(1e-6, self.t_headway)))

        # (4) Right-wall confidence slow-down while D is far/NaN
        if (not math.isfinite(D)) or (D > self.right_present):
            v *= self.reacq_vscale

        return clamp(v, 0.0, self.v_nom)

    def _send_smooth(self, v_des, w_des, dt):
        # Clamp desireds
        w_des = clamp(w_des, -self.w_max, +self.w_max)
        v_des = clamp(v_des, 0.0 if v_des >= 0.0 else -self.v_floor, self.v_nom)

        # Slew v
        dv_max = self.v_slew * dt
        v_cmd = self.v_cmd_prev + clamp(v_des - self.v_cmd_prev, -dv_max, +dv_max)
        if v_cmd > 1e-6:
            v_cmd = max(self.v_floor, v_cmd)

        # Slew w
        dw_max = self.w_slew * dt
        w_cmd = self.w_cmd_prev + clamp(w_des - self.w_cmd_prev, -dw_max, +dw_max)

        self.v_cmd_prev = v_cmd
        self.w_cmd_prev = w_cmd
        self.send(v_cmd, w_cmd)

    # ---------- Live param updates ----------
    def _on_param_change(self, params):
        for p in params:
            if p.name in ('Kp_e','Kd_e','Kp_a','Kd_a','v_nom','w_max'):
                setattr(self, p.name, float(p.value))
            elif p.name == 'desired_right_dist':
                self.d_des = float(p.value)
            elif p.name in (
                'range_ewma_alpha','deriv_alpha','deriv_clip','v_slew','w_slew','v_floor',
                'turn_slowdown','front_slowdown_gain','front_stop','front_clear',
                'right_present_max','acquire_consistency_frames','acquire_right_max',
                'front_turn_start','front_turn_max','left_turn_gain','left_turn_wmax_scale',
                'corner_front_thresh','corner_right_thresh','corner_min_arc_s',
                'corner_v_scale','corner_w_scale','corner_exit_front','corner_exit_band','corner_hysteresis',
                'front_width_deg','right_width_deg','fr_width_deg','left_width_deg',
                'time_headway_s','sensor_guard_frac','sensor_guard_margin_m','reacquire_v_scale'
            ):
                val = float(p.value) if p.name not in ('acquire_consistency_frames',) else int(p.value)
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
                    'acquire_consistency_frames':'need_frames',
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
                    'front_width_deg': None,
                    'right_width_deg': None,
                    'fr_width_deg': None,
                    'left_width_deg': None,
                    'time_headway_s':'t_headway',
                    'sensor_guard_frac':'sensor_guard',
                    'sensor_guard_margin_m':'sensor_margin',
                    'reacquire_v_scale':'reacq_vscale',
                }
                attr = mapname[p.name]
                if attr is None:
                    if p.name == 'front_width_deg': self.front_w = math.radians(val)
                    if p.name == 'right_width_deg': self.right_w = math.radians(val)
                    if p.name == 'fr_width_deg':    self.fr_w    = math.radians(val)
                    if p.name == 'left_width_deg':  self.left_w  = math.radians(val)
                else:
                    setattr(self, attr, val)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothRightFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
