#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

def yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class VisitedMap(Node):
    def __init__(self):
        super().__init__('visited_map')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('visited_topic', '/visited_map')
        self.declare_parameter('overlay_topic', '/lidar_overlay')
        self.declare_parameter('fixed_frame', 'map')
        # Back sector: [135°,180°] U [-180°,-135°]
        self.declare_parameter('back_min_deg', 100.0)
        self.declare_parameter('back_max_deg', 180.0)
        self.declare_parameter('ray_stride', 1)

        # Visualization
        self.declare_parameter('cube_scale_mul', 1.5)  # size relative to map res
        self.declare_parameter('marker_lifetime', 0.0) # 0 = infinite

        # Accumulation resolution (world grid). Default: match map resolution.
        self.declare_parameter('accum_resolution', 0.0)  # 0 -> use map res dynamically

        map_topic     = self.get_parameter('map_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        visited_topic = self.get_parameter('visited_topic').value
        overlay_topic = self.get_parameter('overlay_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # I/O
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.visited_pub = self.create_publisher(OccupancyGrid, visited_topic, 10)
        self.marker_pub  = self.create_publisher(MarkerArray, overlay_topic, 10)

        # State
        self.map_msg  = None
        self.map_grid = None
        self.visited  = None
        self.W = self.H = 0
        self.res = 0.05
        self.ox = self.oy = 0.0

        # Persistent accumulation in WORLD coords (map frame), quantized
        # Each element is an (ix, iy) tuple on a fixed-size world grid.
        self.seen_points_world = set()
        self.accum_res = 0.05  # will be updated on first map

        self.get_logger().info("visited_map with persistent WORLD accumulation ready.")

    # ---------------- Callbacks ----------------

    def map_cb(self, msg: OccupancyGrid):
        """On map update, rebuild the local visited grid but keep world points."""
        self.map_msg = msg
        self.W = msg.info.width
        self.H = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.map_grid = np.asarray(msg.data, dtype=np.int16).reshape(self.H, self.W)

        # pick accumulation resolution
        acc_param = float(self.get_parameter('accum_resolution').value)
        self.accum_res = self.res if acc_param <= 0.0 else acc_param

        # Recreate visited grid for current map dims, but DO NOT clear seen_points_world
        self.visited = np.full((self.H, self.W), -1, dtype=np.int8)  # unknown

        # Paint any already-seen world cells onto the new grid as free
        for ix, iy in self.seen_points_world:
            wx = ix * self.accum_res + 0.5 * self.accum_res
            wy = iy * self.accum_res + 0.5 * self.accum_res
            mx, my = self.world_to_cell(wx, wy)
            if 0 <= mx < self.W and 0 <= my < self.H:
                self.visited[my, mx] = 0

        self.get_logger().info(f"/map: {self.W}x{self.H}, res={self.res:.3f}, origin=({self.ox:.2f},{self.oy:.2f}), accum_res={self.accum_res:.3f}")

    def scan_cb(self, scan: LaserScan):
        if self.map_msg is None or self.visited is None:
            return

        # TF: map -> scan frame
        scan_frame = scan.header.frame_id if scan.header.frame_id else 'base_scan'
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, scan_frame, Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q  = tf.transform.rotation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        except (LookupException, ConnectivityException, ExtrapolationException):
            tx = ty = yaw = 0.0

        # Mark sensor cell free in current visited grid
        rmx, rmy = self.world_to_cell(tx, ty)
        if 0 <= rmx < self.W and 0 <= rmy < self.H:
            self.visited[rmy, rmx] = 0

        # Back sector
        back_min = math.radians(float(self.get_parameter('back_min_deg').value))  # 135°
        back_max = math.radians(float(self.get_parameter('back_max_deg').value))  # 180°
        stride   = max(1, int(self.get_parameter('ray_stride').value))

        # Accumulate new world points
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if i % stride != 0:
                angle += scan.angle_increment
                continue
            if not (math.isfinite(r) and r >= scan.range_min and r <= scan.range_max):
                angle += scan.angle_increment
                continue
            # sector: [135°,180°] U [-180°,-135°]
            if not ((back_min <= angle <= back_max) or (-back_max <= angle <= -back_min)):
                angle += scan.angle_increment
                continue

            # Endpoint in MAP frame
            ex = tx + r * math.cos(yaw + angle)
            ey = ty + r * math.sin(yaw + angle)
            exi, eyi = self.world_to_cell(ex, ey)
            if not (0 <= exi < self.W and 0 <= eyi < self.H):
                angle += scan.angle_increment
                continue

            # Raycast from sensor cell to endpoint
            for cx, cy in self.bresenham(rmx, rmy, exi, eyi):
                if not (0 <= cx < self.W and 0 <= cy < self.H):
                    break
                val = int(self.map_grid[cy, cx])
                if val == 100:      # obstacle -> stop
                    break
                if val == 0:        # mutual FREE -> add to WORLD accumulation
                    wx, wy = self.cell_center(cx, cy)
                    ix, iy = self.quantize_world(wx, wy)
                    self.seen_points_world.add((ix, iy))
                    # also paint current visited grid so it looks good now
                    self.visited[cy, cx] = 0
                # if -1 unknown -> skip but keep marching

            angle += scan.angle_increment

        # Build persistent marker from ALL accumulated world points
        points = []
        for ix, iy in self.seen_points_world:
            wx = ix * self.accum_res + 0.5 * self.accum_res
            wy = iy * self.accum_res + 0.5 * self.accum_res
            points.append(Point(x=wx, y=wy, z=0.0))

        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_mutual_free_world"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        scale_mul = float(self.get_parameter('cube_scale_mul').value)
        marker.scale.x = self.res * scale_mul
        marker.scale.y = self.res * scale_mul
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        life = float(self.get_parameter('marker_lifetime').value)
        marker.lifetime = Duration(sec=int(life), nanosec=int((life - int(life)) * 1e9))
        marker.points = points

        ma = MarkerArray()
        ma.markers.append(marker)

        # Publish map + overlay
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.fixed_frame
        out.info = self.map_msg.info
        out.data = self.visited.flatten().tolist()
        self.visited_pub.publish(out)
        self.marker_pub.publish(ma)

    # ---------------- Helpers ----------------

    def world_to_cell(self, x: float, y: float):
        mx = int(math.floor((x - self.ox) / self.res))
        my = int(math.floor((y - self.oy) / self.res))
        return mx, my

    def cell_center(self, mx: int, my: int):
        x = self.ox + (mx + 0.5) * self.res
        y = self.oy + (my + 0.5) * self.res
        return x, y

    def quantize_world(self, x: float, y: float):
        """Quantize world coords onto accumulation grid independent of map origin."""
        ix = int(math.floor(x / self.accum_res))
        iy = int(math.floor(y / self.accum_res))
        return ix, iy

    def bresenham(self, x0: int, y0: int, x1: int, y1: int):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells


def main(args=None):
    rclpy.init(args=args)
    node = VisitedMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
