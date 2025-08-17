import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def clamp(v, lo, hi): return max(lo, min(hi, v))

class Explorer(Node):
    """
    Open-loop right-hand follower (no P-control):
      INITIAL_RUN  -> straight until front wall
      ALIGN_RIGHT  -> left to put wall on right
      TURN_LEFT    -> bounded left rotation (~85°) with early exit
      FOLLOW_RIGHT -> open-loop band logic (nudge left/right/straight/seek-right)
      BACKOFF      -> brief reverse then ALIGN_RIGHT
    """

    def __init__(self):
        super().__init__('autonomous_explorer')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Speeds
        self.declare_parameter('linear_speed', 0.45)
        self.declare_parameter('angular_speed', 0.6)

        # Distances (m)
        self.declare_parameter('desired_right_dist', 1.20) # 1.2
        self.declare_parameter('right_present_max', 1.70) # 1.7
        self.declare_parameter('front_stop', 1.50) # 0.9
        self.declare_parameter('front_clear_mult', 1.30)
        self.declare_parameter('near_right_thresh', 0.45)

        # Sector widths (deg)
        self.declare_parameter('front_width_deg', 30.0)
        self.declare_parameter('side_width_deg', 40.0)
        self.declare_parameter('diag_width_deg', 30.0)

        # Open-loop band and nudge settings
        self.declare_parameter('right_band', 0.18)
        self.declare_parameter('bias_right_rate_scale', 0.18)   # fraction of angular_speed
        self.declare_parameter('nudge_left_rate_scale', 0.70)   # fraction of angular_speed
        self.declare_parameter('nudge_right_rate_scale', 0.55)  # fraction of angular_speed
        self.declare_parameter('nudge_forward_scale', 0.9)
        self.declare_parameter('mode_dwell_s', 0.25)

        # Backoff
        self.declare_parameter('backoff_time_s', 0.9)

        # Bounded left turn config
        self.declare_parameter('left_turn_angle_deg', 85.0)
        self.declare_parameter('left_turn_rate_scale', 0.65)
        self.declare_parameter('left_turn_max_extra_s', 0.4)

        # NEW: yaw cap & slew (to prevent big, jumpy turns)
        self.declare_parameter('yaw_sat_scale', 0.35 / 2)  # max |yaw| = 0.35 * angular_speed
        self.declare_parameter('yaw_slew_rate', 7.0)   # rad/s^2 max change

        # Read params
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

        self.front_w       = math.radians(float(gp('front_width_deg').value))
        self.side_w        = math.radians(float(gp('side_width_deg').value))
        self.diag_w        = math.radians(float(gp('diag_width_deg').value))

        self.right_band    = float(gp('right_band').value)
        self.bias_right    = float(gp('bias_right_rate_scale').value) * self.v_ang
        self.nudge_left    = float(gp('nudge_left_rate_scale').value) * self.v_ang
        self.nudge_right   = float(gp('nudge_right_rate_scale').value) * self.v_ang
        self.nudge_vscale  = float(gp('nudge_forward_scale').value)
        self.mode_dwell_s  = float(gp('mode_dwell_s').value)

        self.backoff_time  = float(gp('backoff_time_s').value)

        self.left_turn_angle = math.radians(float(gp('left_turn_angle_deg').value))
        self.left_turn_rate  = max(0.1, float(gp('left_turn_rate_scale').value) * self.v_ang)
        self.left_turn_max_extra = float(gp('left_turn_max_extra_s').value)

        self.yaw_sat   = float(gp('yaw_sat_scale').value) * self.v_ang
        self.yaw_slew  = float(gp('yaw_slew_rate').value)

        # I/O
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # State
        self.last_scan = None
        self.state = 'INITIAL_RUN'
        self.state_enter_t = self.now_s()
        self.last_debug_t = 0.0

        # TURN_LEFT timing
        self.turn_end_time = 0.0

        # FOLLOW_RIGHT mode (open-loop)
        self.mode = 'STRAIGHT'   # STRAIGHT | NUDGE_LEFT | NUDGE_RIGHT | SEEK_RIGHT
        self.mode_enter_t = self.now_s()

        # yaw smoothing state
        self.prev_yaw = 0.0
        self.prev_time = self.now_s()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'Right-wall exploration (open-loop, smoothed yaw): scan="{self.scan_topic}", cmd_vel="{self.cmd_vel_topic}"')

    # --- helpers ---
    def now_s(self): return self.get_clock().now().nanoseconds * 1e-9
    def time_in_state(self): return self.now_s() - self.state_enter_t
    def set_state(self, s):
        self.state, self.state_enter_t = s, self.now_s()
        if s == 'TURN_LEFT':
            t_rot = self.left_turn_angle / max(1e-3, self.left_turn_rate)
            self.turn_end_time = self.now_s() + t_rot + self.left_turn_max_extra
            # reset ramp so we don't carry yaw from follow into the discrete turn
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

    def on_scan(self, msg: LaserScan):
        rmin = max(0.0, msg.range_min)
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
        rngs = []
        for r in msg.ranges:
            if r is None or not math.isfinite(r) or r == 0.0 or r < rmin or r > rmax:
                rngs.append(float('inf'))
            else:
                rngs.append(r)
        self.last_scan = (msg.angle_min, msg.angle_increment, rngs)

    def sector_min(self, center_rad, width_rad):
        if self.last_scan is None: return float('inf')
        a0, aincr, rngs = self.last_scan
        n = len(rngs)
        lo = int((center_rad - width_rad/2 - a0) / aincr)
        hi = int((center_rad + width_rad/2 - a0) / aincr)
        lo = clamp(lo, 0, n-1); hi = clamp(hi, 0, n-1)
        if hi < lo: lo, hi = hi, lo
        vals = rngs[lo:hi+1]
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

        now = self.now_s()
        dt  = max(1e-3, now - self.prev_time)
        self.prev_time = now

        if now - self.last_debug_t > 0.6:
            self.get_logger().info(f"state={self.state} mode={self.mode}  F={f:.2f}  R={r:.2f}  L={l:.2f}  FR={fr:.2f}")
            self.last_debug_t = now

        # global safety
        if f < self.d_front_stop * 0.7 and self.state not in ('BACKOFF', 'TURN_LEFT'):
            self.set_state('BACKOFF')

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

            # lost right wall? seek right smoothly (no in-place spin)
            if not self.has_right_wall(r) and f > self.d_front_stop:
                yaw_tgt = -0.35 * self.v_ang  # gentle right
                v_scale = 0.6
            else:
                # Band logic (choose target yaw + speed)
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

            # corner smoothing bias (helps outer corners)
            if fr < max(self.d_right_des, 0.45):
                yaw_tgt += +0.2 * self.v_ang  # small extra left bias

            # cap yaw
            yaw_tgt = clamp(yaw_tgt, -self.yaw_sat, +self.yaw_sat)
            # slew-limit change
            dyaw_max = self.yaw_slew * dt
            yaw_cmd = self.prev_yaw + clamp(yaw_tgt - self.prev_yaw, -dyaw_max, +dyaw_max)
            self.prev_yaw = yaw_cmd

            # small brake when turning hard
            turn_frac = min(1.0, abs(yaw_cmd) / max(1e-6, self.yaw_sat))
            v_scale = max(0.6, v_scale * (1.0 - 0.4 * turn_frac))

            self.forward(v_scale, yaw=yaw_cmd)

        elif self.state == 'TURN_LEFT':
            done = (now >= self.turn_end_time) or (f > f_clear and self.has_right_wall(r))
            if done:
                self.set_state('FOLLOW_RIGHT')
                self.set_mode('STRAIGHT')
                # reset yaw ramp after discrete turn
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
