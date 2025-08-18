#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

def clamp(v, lo, hi): return max(lo, min(hi, v))

class Explorer(Node):
    """
    Right-hand follower with robust PD overlays and a start-mode flag:

      start_with_initial_run = true:
        INITIAL_RUN  -> ALIGN_RIGHT -> FOLLOW_RIGHT
      start_with_initial_run = false:
        FOLLOW_RIGHT immediately (right-hand rule / right-turn policy)

      Other states:
        TURN_LEFT (bounded) and BACKOFF for safety
    """

    def __init__(self):
        super().__init__('autonomous_explorer')

        # --- Topics ---
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # --- Speeds ---
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.2)

        # --- Distances (m) ---
        self.declare_parameter('desired_right_dist', 1.20)
        self.declare_parameter('right_present_max', 1.90)
        self.declare_parameter('front_stop', 1.50)
        self.declare_parameter('front_clear_mult', 1.30)
        self.declare_parameter('near_right_thresh', 0.55)

        # --- Sector widths (deg) ---
        self.declare_parameter('front_width_deg', 30.0)
        self.declare_parameter('side_width_deg', 40.0)
        self.declare_parameter('diag_width_deg', 30.0)

        # --- Open-loop band & nudges ---
        self.declare_parameter('right_band', 0.18)
        self.declare_parameter('bias_right_rate_scale', 0.12)   # fraction of angular_speed
        self.declare_parameter('nudge_left_rate_scale', 0.55)   # fraction of angular_speed
        self.declare_parameter('nudge_right_rate_scale', 0.48)  # fraction of angular_speed
        self.declare_parameter('nudge_forward_scale', 0.9)
        self.declare_parameter('mode_dwell_s', 0.18)

        # --- Backoff ---
        self.declare_parameter('backoff_time_s', 0.7)

        # --- Bounded left turn config ---
        self.declare_parameter('left_turn_angle_deg', 80.0)
        self.declare_parameter('left_turn_rate_scale', 0.75)
        self.declare_parameter('left_turn_max_extra_s', 0.3)

        # --- Yaw cap & slew ---
        self.declare_parameter('yaw_sat_scale', 0.38)   # |yaw| <= yaw_sat_scale * angular_speed
        self.declare_parameter('yaw_slew_rate', 14.0)   # rad/s^2

        # --- PD gains ---
        self.declare_parameter('Kp_yaw', 0.8)
        self.declare_parameter('Kd_yaw', 0.25)
        self.declare_parameter('Kp_v', 0.9)             # m/s per m
        self.declare_parameter('Kd_v', 0.35)            # m/s per (m/s)
        self.declare_parameter('v_pd_down_max', 1.2)    # max decel from PD (m/s)

        # --- PD stabilizers ---
        self.declare_parameter('pd_error_alpha', 0.30)  # error EWMA
        self.declare_parameter('deriv_alpha', 0.50)     # derivative EWMA
        self.declare_parameter('deriv_clip', 5.0)       # limit |de/dt|
        self.declare_parameter('yaw_deadband_m', 0.03)  # wall error deadband
        self.declare_parameter('vel_deadband_m', 0.10)  # front error deadband

        # --- Start-mode flag (NEW) ---
        self.declare_parameter('start_with_initial_run', False)

        # ---------- Read params ----------
        gp = self.get_parameter
        self.scan_topic    = gp('scan_topic').value
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.v_lin         = float(gp('linear_speed').value)
        self.v_ang         = float(gp('angular_speed').value)

        self.d_right_des   = float(gp('desired_right_dist').value)
        self.right_present = float(gp('right_present_max').value)
        self.d_front_stop  = float(gp('front_stop').value)
        self.front_clear_mult = float(gp('front_clear_mult').value)
        self.near_right_thresh = float(gp('near_right_thresh').value)

        self.front_w = math.radians(float(gp('front_width_deg').value))
        self.side_w  = math.radians(float(gp('side_width_deg').value))
        self.diag_w  = math.radians(float(gp('diag_width_deg').value))

        self.right_band   = float(gp('right_band').value)
        self.bias_right   = float(gp('bias_right_rate_scale').value) * self.v_ang
        self.nudge_left   = float(gp('nudge_left_rate_scale').value) * self.v_ang
        self.nudge_right  = float(gp('nudge_right_rate_scale').value) * self.v_ang
        self.nudge_vscale = float(gp('nudge_forward_scale').value)
        self.mode_dwell_s = float(gp('mode_dwell_s').value)

        self.backoff_time  = float(gp('backoff_time_s').value)

        self.left_turn_angle     = math.radians(float(gp('left_turn_angle_deg').value))
        self.left_turn_rate      = max(0.1, float(gp('left_turn_rate_scale').value) * self.v_ang)
        self.left_turn_max_extra = float(gp('left_turn_max_extra_s').value)

        self.yaw_sat   = float(gp('yaw_sat_scale').value) * self.v_ang
        self.yaw_slew  = float(gp('yaw_slew_rate').value)

        # PD params
        self.Kp_yaw = float(gp('Kp_yaw').value)
        self.Kd_yaw = float(gp('Kd_yaw').value)
        self.Kp_v   = float(gp('Kp_v').value)
        self.Kd_v   = float(gp('Kd_v').value)
        self.v_pd_down_max = float(gp('v_pd_down_max').value)

        self.pd_error_alpha = float(gp('pd_error_alpha').value)
        self.deriv_alpha    = float(gp('deriv_alpha').value)
        self.deriv_clip     = float(gp('deriv_clip').value)
        self.yaw_deadband_m = float(gp('yaw_deadband_m').value)
        self.vel_deadband_m = float(gp('vel_deadband_m').value)

        self.start_with_initial_run = bool(gp('start_with_initial_run').value)

        # ---------- I/O ----------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ---------- State ----------
        self.last_scan = None
        self.last_debug_t = 0.0
        self.turn_end_time = 0.0

        # FOLLOW_RIGHT mode bookkeeping
        self.mode = 'STRAIGHT'   # STRAIGHT | NUDGE_LEFT | NUDGE_RIGHT | SEEK_RIGHT
        self.mode_enter_t = self.now_s()

        # yaw smoothing state & timing
        self.prev_yaw = 0.0
        self.prev_time = self.now_s()

        # PD memory (filtered errors & derivatives)
        self.prev_e_yaw = 0.0
        self.prev_e_v   = 0.0
        self.e_yaw_f = 0.0
        self.e_v_f   = 0.0
        self.de_yaw_f = 0.0
        self.de_v_f   = 0.0

        # Start state (controlled by flag)
        if self.start_with_initial_run:
            self.state = 'INITIAL_RUN'
        else:
            self.state = 'FOLLOW_RIGHT'
            self.mode = 'STRAIGHT'
        self.state_enter_t = self.now_s()

        # Control timer
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Explorer (PD) start_with_initial_run={self.start_with_initial_run}  scan="{self.scan_topic}" cmd="{self.cmd_vel_topic}"'
        )

        # Register PD + flag hot-reload callback
        self.add_on_set_parameters_callback(self._on_param_change)

    # --- helpers ---
    def now_s(self): return self.get_clock().now().nanoseconds * 1e-9
    def time_in_state(self): return self.now_s() - self.state_enter_t
    def set_state(self, s):
        self.state, self.state_enter_t = s, self.now_s()
        if s == 'TURN_LEFT':
            t_rot = self.left_turn_angle / max(1e-3, self.left_turn_rate)
            self.turn_end_time = self.now_s() + t_rot + self.left_turn_max_extra
            self.prev_yaw = 0.0
            self.prev_time = self.now_s()

    def set_mode(self, m):
        if m != self.mode:
            self.mode = m
            self.mode_enter_t = self.now_s()

    def mode_dwell_ok(self): return (self.now_s() - self.mode_enter_t) >= self.mode_dwell_s

    def send(self, v, wz):
        msg = Twist(); msg.linear.x = v; msg.angular.z = clamp(wz, -self.v_ang, self.v_ang)
        self.cmd_pub.publish(msg)
    def forward(self, scale=1.0, yaw=0.0): self.send(self.v_lin*scale, yaw)
    def turn_left(self, rate=None):         self.send(0.0, + (self.left_turn_rate if rate is None else rate))
    def turn_right(self, scale=1.0):        self.send(0.0, -self.v_ang*scale)
    def backoff(self):                      self.send(-0.12, 0.0)
    def stop(self):                         self.send(0.0, 0.0)

    # --- scan handling ---
    def on_scan(self, msg: LaserScan):
        rmin = max(0.0, msg.range_min)
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
        rngs = []
        for r in msg.ranges:
            if r is None or not math.isfinite(r) or r == 0.0 or r < rmin or r > rmax:
                rngs.append(float('inf'))
            else:
                rngs.append(r)
        self.last_scan = (msg.angle_min, msg.angle_increment, rngs, rmax)

    def sector_min(self, center_rad, width_rad):
        """Min range over a sector, robust to angle wrap (0..2π)."""
        if self.last_scan is None: return float('inf')
        a0, aincr, rngs, rmax = self.last_scan
        n = len(rngs)
        if n == 0 or aincr == 0.0: return float('inf')

        span = n * aincr
        start = center_rad - width_rad/2.0
        stop  = center_rad + width_rad/2.0

        def ang_to_idx(theta):
            off = (theta - a0) % span
            return int(clamp(math.floor(off / aincr), 0, n-1))

        i_lo = ang_to_idx(start)
        i_hi = ang_to_idx(stop)

        if i_hi >= i_lo:
            vals = rngs[i_lo:i_hi+1]
        else:
            vals = rngs[i_lo:] + rngs[:i_hi+1]

        vals = [v if (v is not None and math.isfinite(v)) else rmax for v in vals]
        return min(vals) if vals else float('inf')

    # Angles: 0=front, -pi/2=right, +pi/2=left, -pi/4=front-right
    def d_front(self):        return self.sector_min(0.0, self.front_w)
    def d_right(self):        return self.sector_min(-math.pi/2, self.side_w)
    def d_left(self):         return self.sector_min(+math.pi/2, self.side_w)
    def d_front_right(self):  return self.sector_min(-math.pi/4, self.diag_w)

    def has_right_wall(self, dr): return math.isfinite(dr) and (dr <= self.right_present)
    def is_dead_end(self, f, r, l):
        close = self.d_front_stop
        return (f < close) and (r < close) and (l < close)

    # --- main loop ---
    def control_loop(self):
        if self.last_scan is None:
            self.turn_left(rate=self.left_turn_rate*0.5)
            return

        f  = self.d_front()
        r  = self.d_right()
        l  = self.d_left()
        fr = self.d_front_right()
        f_clear = self.d_front_stop * self.front_clear_mult

        # guards for PD
        safe_far = max(3.0, self.d_front_stop * self.front_clear_mult * 2.0)
        if not (f is not None and math.isfinite(f)): f = safe_far
        if not (r is not None and math.isfinite(r)): r = safe_far

        now = self.now_s()
        dt  = max(1e-3, now - self.prev_time)
        self.prev_time = now

        if now - self.last_debug_t > 0.6:
            self.get_logger().info(f"state={self.state} mode={self.mode}  F={f:.2f}  R={r:.2f}  L={l:.2f}  FR={fr:.2f}")
            self.last_debug_t = now

        # global safety
        if f < self.d_front_stop * 0.7 and self.state not in ('BACKOFF', 'TURN_LEFT'):
            self.set_state('BACKOFF')

        # --- state machine ---
        if self.state == 'INITIAL_RUN':
            if f < self.d_front_stop:
                self.set_state('ALIGN_RIGHT')
            else:
                self.forward()

        elif self.state == 'ALIGN_RIGHT':
            if self.has_right_wall(r) and f > f_clear:
                self.set_state('FOLLOW_RIGHT')
                self.set_mode('STRAIGHT')
            else:
                self.turn_left()

        elif self.state == 'FOLLOW_RIGHT':
            # exits
            if self.is_dead_end(f, r, l) or r < self.near_right_thresh or f < self.d_front_stop:
                self.set_state('TURN_LEFT'); return

            # if no right wall yet and front is open, seek right smoothly
            if not self.has_right_wall(r) and f > self.d_front_stop:
                yaw_tgt = -0.35 * self.v_ang  # gentle right bias to pick up right wall
                v_scale = 0.6
            else:
                # band logic around desired right distance
                low  = self.d_right_des - self.right_band
                high = self.d_right_des + self.right_band

                if r < low and (self.mode_dwell_ok() or self.mode != 'NUDGE_LEFT'):
                    self.set_mode('NUDGE_LEFT')
                elif r > high and (self.mode_dwell_ok() or self.mode != 'NUDGE_RIGHT'):
                    self.set_mode('NUDGE_RIGHT')
                elif low <= r <= high and (self.mode_dwell_ok() or self.mode != 'STRAIGHT'):
                    self.set_mode('STRAIGHT')

                if self.mode == 'NUDGE_LEFT':
                    yaw_tgt = +self.nudge_left
                    v_scale = self.nudge_vscale
                elif self.mode == 'NUDGE_RIGHT':
                    yaw_tgt = -self.nudge_right
                    v_scale = self.nudge_vscale
                else:  # STRAIGHT
                    yaw_tgt = -self.bias_right
                    v_scale = 1.0

            # --- Robust PD overlays ---

            # Angular PD on right-wall error
            e_yaw_raw = (self.d_right_des - r)
            if abs(e_yaw_raw) < self.yaw_deadband_m:
                e_yaw_raw = 0.0
            a = self.pd_error_alpha
            b = self.deriv_alpha
            self.e_yaw_f = (1.0 - a) * self.e_yaw_f + a * e_yaw_raw
            de_yaw_raw = (self.e_yaw_f - self.prev_e_yaw) / dt
            self.de_yaw_f = (1.0 - b) * self.de_yaw_f + b * de_yaw_raw
            self.de_yaw_f = clamp(self.de_yaw_f, -self.deriv_clip, +self.deriv_clip)
            yaw_tgt += self.Kp_yaw * self.e_yaw_f + self.Kd_yaw * self.de_yaw_f
            self.prev_e_yaw = self.e_yaw_f

            # corner smoothing bias
            if fr < max(self.d_right_des, 0.45):
                yaw_tgt += +0.2 * self.v_ang

            # cap & slew-limit
            yaw_tgt = clamp(yaw_tgt, -self.yaw_sat, +self.yaw_sat)
            dyaw_max = self.yaw_slew * dt
            yaw_cmd = self.prev_yaw + clamp(yaw_tgt - self.prev_yaw, -dyaw_max, +dyaw_max)
            self.prev_yaw = yaw_cmd

            # speed reduction when turning hard
            turn_frac = min(1.0, abs(yaw_cmd) / max(1e-6, self.yaw_sat))
            v_scale = max(0.6, v_scale * (1.0 - 0.4 * turn_frac))

            # Linear PD: decel toward front clearance
            v_cmd = self.v_lin * v_scale
            e_v_raw = (f - f_clear)
            if abs(e_v_raw) < self.vel_deadband_m:
                e_v_raw = 0.0
            self.e_v_f = (1.0 - a) * self.e_v_f + a * e_v_raw
            de_v_raw = (self.e_v_f - self.prev_e_v) / dt
            self.de_v_f = (1.0 - b) * self.de_v_f + b * de_v_raw
            self.de_v_f = clamp(self.de_v_f, -self.deriv_clip, +self.deriv_clip)
            v_pd = self.Kp_v * self.e_v_f + self.Kd_v * self.de_v_f
            v_pd = min(0.0, v_pd)
            v_pd = max(-self.v_pd_down_max, v_pd)
            v_cmd = clamp(v_cmd + v_pd, 0.0, self.v_lin)

            self.prev_e_v = self.e_v_f
            self.send(v_cmd, yaw_cmd)

        elif self.state == 'TURN_LEFT':
            done = (now >= self.turn_end_time) or (f > f_clear and self.has_right_wall(r))
            if done:
                self.set_state('FOLLOW_RIGHT')
                self.set_mode('STRAIGHT')
                self.prev_yaw = 0.0
                self.prev_time = self.now_s()
                return
            self.turn_left()

        elif self.state == 'BACKOFF':
            if self.time_in_state() < self.backoff_time:
                self.backoff()
            else:
                self.stop()
                self.set_state('ALIGN_RIGHT')

    # --- live param changes: PD + start flag ---
    def _on_param_change(self, params):
        changed_yaw = False
        changed_vel = False
        switch_mode  = None

        for p in params:
            if p.name == 'Kp_yaw':
                self.Kp_yaw = float(p.value); changed_yaw = True
            elif p.name == 'Kd_yaw':
                self.Kd_yaw = float(p.value); changed_yaw = True
            elif p.name == 'Kp_v':
                self.Kp_v = float(p.value); changed_vel = True
            elif p.name == 'Kd_v':
                self.Kd_v = float(p.value); changed_vel = True
            elif p.name == 'v_pd_down_max':
                self.v_pd_down_max = float(p.value)
            elif p.name == 'pd_error_alpha':
                self.pd_error_alpha = float(p.value)
            elif p.name == 'deriv_alpha':
                self.deriv_alpha = float(p.value)
            elif p.name == 'deriv_clip':
                self.deriv_clip = float(p.value)
            elif p.name == 'yaw_deadband_m':
                self.yaw_deadband_m = float(p.value)
            elif p.name == 'vel_deadband_m':
                self.vel_deadband_m = float(p.value)

            # NEW: hot-switch start mode
            elif p.name == 'start_with_initial_run':
                new_flag = bool(p.value)
                if new_flag != self.start_with_initial_run:
                    self.start_with_initial_run = new_flag
                    if not new_flag and self.state in ('INITIAL_RUN', 'ALIGN_RIGHT'):
                        switch_mode = 'FOLLOW_RIGHT'

        if changed_yaw:
            self.de_yaw_f = 0.0
            self.prev_e_yaw = self.e_yaw_f
        if changed_vel:
            self.de_v_f = 0.0
            self.prev_e_v = self.e_v_f

        if switch_mode == 'FOLLOW_RIGHT':
            self.set_state('FOLLOW_RIGHT')
            self.set_mode('STRAIGHT')
            self.get_logger().info('Switched to FOLLOW_RIGHT due to start_with_initial_run=false')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
