#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quat(x, y, z, w) -> float:
    # ZYX yaw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WindowFreeRatioVisited(Node):
    def __init__(self):
        super().__init__('window_free_ratio_visited')

        # --- Parameters ---
        self.declare_parameter('visited_topic', '/visited_map')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('window_area_m2', 20.0 * np.pi)
        self.declare_parameter('window_shape', 'circle')   # 'circle' or 'square'

        visited_topic = self.get_parameter('visited_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # --- I/O ---
        self.visited_sub = self.create_subscription(OccupancyGrid, visited_topic, self.visited_cb, 10)
        self.free_ratio_pub = self.create_publisher(Float32, '/free_ratio_visited', 10)
        self.occ_ratio_pub  = self.create_publisher(Float32, '/occ_ratio_visited', 10)

        self.centroid_pub            = self.create_publisher(PointStamped, '/free_centroid', 10)
        self.centroid_angle_map_pub  = self.create_publisher(Float32, '/free_centroid_angle_map_deg', 10)
        self.centroid_angle_robot_pub= self.create_publisher(Float32, '/free_centroid_angle_robot_deg', 10)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ---
        self.visited_msg = None
        self.timer = self.create_timer(0.1, self.tick)  # 5 Hz

        self.get_logger().info('window_free_ratio_visited ready (source=/visited_map).')

    def visited_cb(self, msg: OccupancyGrid):
        self.visited_msg = msg

    def tick(self):
        if self.visited_msg is None:
            return

        # Robot pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_frame, Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            q  = tf.transform.rotation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        free_ratio, occ_ratio, centroid_xy = self.compute_ratios_and_centroid(rx, ry)
        if free_ratio is None:
            return

        self.free_ratio_pub.publish(Float32(data=free_ratio))
        self.occ_ratio_pub.publish(Float32(data=occ_ratio))

        if centroid_xy is not None:
            cx, cy = centroid_xy
            ps = PointStamped()
            ps.header.frame_id = self.fixed_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point.x = cx; ps.point.y = cy; ps.point.z = 0.0
            self.centroid_pub.publish(ps)

            ang_map = math.degrees(math.atan2(cy - ry, cx - rx))
            ang_robot = math.degrees(self.ang_norm(math.atan2(cy - ry, cx - rx) - yaw))

            self.centroid_angle_map_pub.publish(Float32(data=ang_map))
            self.centroid_angle_robot_pub.publish(Float32(data=ang_robot))

            self.get_logger().info(
                f"FREE={free_ratio:.2%} | OCC={occ_ratio:.2%} | centroid=({cx:.2f},{cy:.2f}) "
                f"| ang_map={ang_map:.1f}° | ang_robot={ang_robot:.1f}°"
            )
        else:
            self.centroid_angle_map_pub.publish(Float32(data=float('nan')))
            self.centroid_angle_robot_pub.publish(Float32(data=float('nan')))
            self.get_logger().info(f"FREE={free_ratio:.2%} | OCC={occ_ratio:.2%} | centroid=None")

    def compute_ratios_and_centroid(self, wx: float, wy: float):
        """
        Interprets /visited_map as:
          FREE (visited) = (cell value != 0)
          OCC/Unknown    = (cell value == 0)
        Returns: (free_ratio, occ_ratio, (cx_world, cy_world) or None)
        """
        msg = self.visited_msg
        res = msg.info.resolution
        W   = msg.info.width
        H   = msg.info.height
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        grid = np.asarray(msg.data, dtype=np.int16).reshape(H, W)

        window_area = float(self.get_parameter('window_area_m2').value)
        shape       = str(self.get_parameter('window_shape').value).lower()

        cx = int(math.floor((wx - ox) / res))
        cy = int(math.floor((wy - oy) / res))
        if not (0 <= cx < W and 0 <= cy < H):
            return None, None, None

        total = 0
        free  = 0
        occ   = 0
        sum_x = 0.0
        sum_y = 0.0

        if shape == 'circle':
            radius_m = math.sqrt(max(0.0, window_area) / math.pi)
            r_cells  = int(math.ceil(radius_m / res))
            min_x = max(0, cx - r_cells); max_x = min(W - 1, cx + r_cells)
            min_y = max(0, cy - r_cells); max_y = min(H - 1, cy + r_cells)
            r2 = r_cells * r_cells

            for y in range(min_y, max_y + 1):
                dyc = y - cy
                for x in range(min_x, max_x + 1):
                    dxc = x - cx
                    if dxc*dxc + dyc*dyc > r2:
                        continue
                    total += 1
                    v = int(grid[y, x])
                    if v != 0:
                        free += 1
                        wx_cell = ox + (x + 0.5) * res
                        wy_cell = oy + (y + 0.5) * res
                        sum_x += wx_cell; sum_y += wy_cell
                    else:
                        occ += 1
        else:
            side_m = math.sqrt(max(0.0, window_area))
            half_cells = int(math.ceil((0.5 * side_m) / res))
            min_x = max(0, cx - half_cells); max_x = min(W - 1, cx + half_cells)
            min_y = max(0, cy - half_cells); max_y = min(H - 1, cy + half_cells)

            sub = grid[min_y:max_y+1, min_x:max_x+1]
            total = int(sub.size)
            free  = int(np.count_nonzero(sub != 0))
            occ   = total - free

            ys, xs = np.nonzero(sub != 0)
            if ys.size > 0:
                xs_full = min_x + xs
                ys_full = min_y + ys
                wx_cells = ox + (xs_full + 0.5) * res
                wy_cells = oy + (ys_full + 0.5) * res
                sum_x = float(np.sum(wx_cells))
                sum_y = float(np.sum(wy_cells))

        denom = total if total > 0 else 1
        free_ratio = free / float(denom)
        occ_ratio  = occ  / float(denom)

        if free > 0:
            cx_world = sum_x / float(free)
            cy_world = sum_y / float(free)
            return free_ratio, occ_ratio, (cx_world, cy_world)
        else:
            return free_ratio, occ_ratio, None

    @staticmethod
    def ang_norm(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = WindowFreeRatioVisited()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
