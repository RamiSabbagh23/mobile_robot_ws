import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class RightWallFollower(Node):
    """
    Pure right-hand wall follower:
      - Keeps a wall on the robot's right.
      - If front is too close -> turn left in place.
      - If right side is too far (no wall) -> steer right to reacquire.
      - Else -> forward with P control on right distance.
    Angle centers (front/right/diag-right) are configurable for your scan convention.
    """

    def __init__(self):
        super().__init__('right_wall_follower')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Motion
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 1.0)

        # Distances (meters)
        self.declare_parameter('desired_right_dist', 0.55)
        self.declare_parameter('stop_front', 0.35)
        self.declare_parameter('grab_right', 0.80)

        # Sector widths (deg) and centers (deg)
        self.declare_parameter('front_width_deg', 30.0)
        self.declare_parameter('side_width_deg', 40.0)
        self.declare_parameter('diag_width_deg', 30.0)

        self.declare_parameter('front_center_deg', 0.0)
        self.declare_parameter('right_center_deg', -90.0)
        self.declare_parameter('diag_center_deg', -45.0)

        # Control
        self.declare_parameter('k_p', 1.1)
        self.declare_parameter('bias_corner', 0.4)
        self.declare_parameter('max_yaw_rate_scale', 1.0)

        # Read params
        gp = self.get_parameter
        self.scan_topic = gp('scan_topic').value
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.v_lin = float(gp('linear_speed').value)
        self.v_ang_base = float(gp('angular_speed').value)
        self.v_ang_scale = float(gp('max_yaw_rate_scale').value)
        
        # Calculate max angular speed after all parameters are read
        self.v_ang = max(0.1, self.v_ang_base * self.v_ang_scale)

        self.d_right_des = float(gp('desired_right_dist').value)
        self.d_stop_front = float(gp('stop_front').value)
        self.d_grab_right = float(gp('grab_right').value)

        self.front_w = math.radians(float(gp('front_width_deg').value))
        self.side_w = math.radians(float(gp('side_width_deg').value))
        self.diag_w = math.radians(float(gp('diag_width_deg').value))

        self.front_c = math.radians(float(gp('front_center_deg').value))
        self.right_c = math.radians(float(gp('right_center_deg').value))
        self.diag_c = math.radians(float(gp('diag_center_deg').value))

        self.k_p = float(gp('k_p').value)
        self.bias_corner = float(gp('bias_corner').value)

        # I/O
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.last_scan = None
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info(f'Right-wall follower on {self.scan_topic} -> {self.cmd_vel_topic}')

    # ---------- utils ----------
    def drive(self, v, wz):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = clamp(wz, -self.v_ang, self.v_ang)
        self.cmd_pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        # Pre-filter into [range_min, range_max] and replace bad with inf
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
        if self.last_scan is None:
            return float('inf')
        a0, aincr, rngs = self.last_scan
        n = len(rngs)
        lo = int((center_rad - width_rad / 2 - a0) / aincr)
        hi = int((center_rad + width_rad / 2 - a0) / aincr)
        lo = clamp(lo, 0, n - 1)
        hi = clamp(hi, 0, n - 1)
        if hi < lo:
            lo, hi = hi, lo
        vals = rngs[lo:hi + 1]
        return min(vals) if vals else float('inf')

    # Convenience sectors using configured centers
    def d_front(self):
        return self.sector_min(self.front_c, self.front_w)

    def d_right(self):
        return self.sector_min(self.right_c, self.side_w)

    def d_front_right(self):
        return self.sector_min(self.diag_c, self.diag_w)

    # ---------- main loop ----------
    def loop(self):
        if self.last_scan is None:
            # rotate gently to collect first scan
            self.drive(0.0, +0.4 * self.v_ang)
            return

        df = self.d_front()
        dr = self.d_right()
        dfr = self.d_front_right()

        # 1) Hard safety: front too close -> stop and turn left
        if df < self.d_stop_front:
            self.drive(0.0, +self.v_ang)
            return

        # 2) If right is far (no wall), steer right to pick it up
        if dr > self.d_grab_right:
            extra = 0.4 if dfr > self.d_grab_right else 0.0
            self.drive(self.v_lin * 0.7, -(0.6 * self.v_ang + extra * self.v_ang))
            return

        # 3) Regular wall-follow using P-control on right distance
        err = (self.d_right_des - dr)
        wz = clamp(-self.k_p * err, -self.v_ang, self.v_ang)

        # corner rounding (if front-right is tight, nudge left)
        if dfr < max(self.d_right_des, 0.45):
            wz += self.bias_corner

        self.drive(self.v_lin, wz)

def main(args=None):
    rclpy.init(args=args)
    node = RightWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

