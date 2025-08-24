#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* path planner (portable src/ layout) + overlay with outside-only safety margin.

Default folders (no PC-specific paths needed):
  - Maps: <ws>/src/autonomous_explorer/map
  - Saved paths & overlays: <ws>/src/autonomous_explorer/path
These are auto-created. You can override with params:
  - map_yaml_path (string) : explicit YAML path
  - map_folder_abs (string): folder to scan for newest YAML
  - save_folder_abs (string): folder to save CSV/YAML/PNG

Start pose sources (priority):
  1) TF map->base_link
  2) TF map->odom + /odom (nav_msgs/Odometry)
  3) /pose (geometry_msgs/PoseStamped in "map")
  4) start_x/start_y parameters

Inflation radius = robot_radius_m + safety_margin_m.

Overlay:
  - magenta: outside edge of inflated region (free cells adjacent to inflated cells)
  - red: path
  - green: start
  - blue: goal
"""

import os, time, math, glob, heapq
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from builtin_interfaces.msg import Time as RosTime
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# Try to locate install/share to backtrack to <ws>/src if possible
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

# Optional image loaders / plotting
try:
    from PIL import Image  # type: ignore
except Exception:
    Image = None

try:
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False


# ---------- tiny utils ----------
def now_msg(node: Node) -> RosTime:
    return node.get_clock().now().to_msg()

def read_yaml_simple(path: str) -> dict:
    try:
        import yaml  # type: ignore
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        data = {}
        with open(path, 'r') as f:
            for ln in f:
                s = ln.strip()
                if (not s) or s.startswith('#') or (':' not in s):
                    continue
                k, v = s.split(':', 1)
                k, v = k.strip(), v.strip()
                if v.startswith('[') and v.endswith(']'):
                    parts = [p.strip() for p in v[1:-1].split(',')]
                    try:
                        data[k] = [float(p) for p in parts]
                    except:
                        data[k] = parts
                else:
                    try:
                        data[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
                    except:
                        data[k] = v.strip('"').strip("'")
        return data

def load_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Unsupported PGM: {magic}, need P5')
        def _tok():
            t = f.readline()
            while t.strip().startswith(b'#'):
                t = f.readline()
            return t
        dims = _tok().split()
        if len(dims) < 2:
            dims += _tok().split()
        w, h = int(dims[0]), int(dims[1])
        maxval = int(_tok())
        img = np.frombuffer(f.read(w*h), dtype=np.uint8).reshape((h, w))
        if maxval != 255:
            img = (img.astype(np.float32) * (255.0/maxval)).astype(np.uint8)
        return img

def load_image_any(path: str) -> np.ndarray:
    if path.lower().endswith('.pgm'):
        return load_pgm(path)
    if Image is None:
        raise RuntimeError("Install pillow to read non-PGM images.")
    return np.array(Image.open(path).convert('L'))

def newest_yaml_in(folder: str) -> str:
    cands = sorted(
        glob.glob(os.path.join(folder, '*.yaml')) + glob.glob(os.path.join(folder, '*.yml')),
        key=os.path.getmtime, reverse=True
    )
    if not cands:
        raise FileNotFoundError(f"No YAML in {folder}")
    return cands[0]

def build_disk_offsets(r: int):
    offs = []
    r2 = r*r
    for dy in range(-r, r+1):
        for dx in range(-r, r+1):
            if dx*dx + dy*dy <= r2:
                offs.append((dx, dy))
    return offs


# ---------- main node ----------
class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # Package constants for portable src/ layout
        self.package_name = 'autonomous_explorer'
        self.map_folder_rel = 'map'   # matches your existing folder name
        self.path_folder_rel = 'path' # where we save outputs

        # Params
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('map_folder_abs', '')     # optional override
        self.declare_parameter('save_folder_abs', '')    # optional override
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('robot_radius_m', 0.10)
        self.declare_parameter('safety_margin_m', 0.25)
        self.declare_parameter('unknown_is_free', False)
        self.declare_parameter('publish_path_topic', '/astar_path')
        self.declare_parameter('tf_wait_s', 3.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        # Save options
        self.declare_parameter('save_path', True)
        self.declare_parameter('save_format', 'both')  # 'csv' | 'yaml' | 'both'
        self.declare_parameter('save_basename', '')    # if empty -> <map_stem>_astar_path

        topic = self.get_parameter('publish_path_topic').get_parameter_value().string_value
        self.path_pub = self.create_publisher(Path, topic, 1)

        # TF + fallbacks
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom = None
        self.last_pose_map = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/pose', self._pose_cb, 10)

        # Resolve folders (prefer <ws>/src/<pkg>/<rel>, then share/<pkg>/<rel>, then cwd/<rel>)
        self.map_dir = self._resolve_package_dir(self.map_folder_rel,
            self.get_parameter('map_folder_abs').get_parameter_value().string_value)
        self.save_dir = self._resolve_package_dir(self.path_folder_rel,
            self.get_parameter('save_folder_abs').get_parameter_value().string_value)

        # Load map YAML + image
        yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        if not yaml_path:
            yaml_path = newest_yaml_in(self.map_dir)
        yaml_path = os.path.abspath(yaml_path)
        self.map_stem = os.path.splitext(os.path.basename(yaml_path))[0]

        meta = read_yaml_simple(yaml_path)
        img_path = os.path.join(os.path.dirname(yaml_path), str(meta['image']))
        self.res = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.ox, self.oy = float(origin[0]), float(origin[1])
        negate = int(meta.get('negate', 0))
        occ_th = float(meta.get('occupied_thresh', 0.65))
        free_th = float(meta.get('free_thresh', 0.196))

        self.img_gray = load_image_any(img_path)  # HxW
        H, W = self.img_gray.shape

        # ----- Occupancy (base, no inflation) -----
        # Occupancy (base, no inflation)
        norm = self.img_gray.astype(np.float32) / 255.0
        occ_prob = (1.0 - norm) if negate == 0 else norm
        unknown_mask = (occ_prob > free_th) & (occ_prob < occ_th)

        self.unknown_mask_base = unknown_mask.copy()   # <-- KEEP THIS

        if not self.get_parameter('unknown_is_free').get_parameter_value().bool_value:
            base_occ = (occ_prob >= occ_th) | unknown_mask
        else:
            base_occ = (occ_prob >= occ_th)

# Inflate base_occ -> self.occ_inflated ...

        # ----- Inflate: robot + safety margin -----
        r_robot = float(self.get_parameter('robot_radius_m').get_parameter_value().double_value)
        r_margin = float(self.get_parameter('safety_margin_m').get_parameter_value().double_value)
        total_r = max(0.0, r_robot + r_margin)
        r_cells = int(round(total_r / self.res))

        inflated_occ = base_occ.copy()
        if r_cells > 0:
            offs = build_disk_offsets(r_cells)
            ys, xs = np.where(base_occ)
            for dx, dy in offs:
                x2 = xs + dx
                y2 = ys + dy
                valid = (x2 >= 0) & (x2 < W) & (y2 >= 0) & (y2 < H)
                inflated_occ[y2[valid], x2[valid]] = True

        self.W, self.H = W, H
        self.occ_base = base_occ           # original obstacles
        self.occ_inflated = inflated_occ   # inflated no-go region
        self.occ = self.occ_inflated       # planner uses inflated
        self.inflation_cells = r_cells
        self.get_logger().info(
            f"Loaded map {os.path.basename(yaml_path)} size=({W}x{H}) res={self.res:.3f}m "
            f"inflation={r_cells} cells (~{total_r:.2f} m)"
        )

        # Run once and exit
        self.plan_and_publish()
        self.create_timer(0.7, self._shutdown)

    # ---------- folder resolution ----------
    def _resolve_package_dir(self, rel_subdir: str, abs_override: str = "") -> str:
        """
        Prefer <ws>/src/<package>/<rel_subdir>.
        If abs_override is provided, use that.
        Else, try deriving <ws> from share dir:
           <ws>/install/<pkg>/share/<pkg>  ->  <ws>/src/<pkg>/<rel_subdir>
        Fallback to <share>/<pkg>/<rel_subdir>.
        Last resort: <cwd>/<rel_subdir>.
        Ensures the directory exists.
        """
        # 1) Absolute override
        p = str(abs_override).strip()
        if p:
            if not os.path.isabs(p):
                p = os.path.abspath(os.path.join(os.getcwd(), p))
            os.makedirs(p, exist_ok=True)
            return p

        # 2) Try to locate share and backtrack to <ws>/src
        share_dir = None
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory(self.package_name)
            except Exception:
                share_dir = None

        if share_dir is not None:
            install_pkg = os.path.dirname(os.path.dirname(share_dir))   # .../install/<pkg>
            install_root = os.path.dirname(install_pkg)                 # .../install
            ws_root = os.path.dirname(install_root)                     # .../<ws>
            candidate_src = os.path.join(ws_root, 'src', self.package_name)
            if os.path.isdir(candidate_src):
                subdir = os.path.join(candidate_src, rel_subdir)
                os.makedirs(subdir, exist_ok=True)
                return subdir
            # fallback to share/<pkg>/<rel_subdir>
            subdir = os.path.join(share_dir, rel_subdir)
            os.makedirs(subdir, exist_ok=True)
            return subdir

        # 3) Last resort: cwd/<rel_subdir>
        base_dir = os.getcwd()
        self.get_logger().warn(f"[autosave] Could not resolve share/src paths. Using CWD: {base_dir}")
        subdir = os.path.join(base_dir, rel_subdir)
        os.makedirs(subdir, exist_ok=True)
        return subdir

    # ---------- subs ----------
    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def _pose_cb(self, msg: PoseStamped):
        self.last_pose_map = msg if msg.header.frame_id == 'map' else None

    # ---------- grid/world ----------
    def world_to_grid(self, x, y):
        cx = (x - self.ox)/self.res
        cy = (y - self.oy)/self.res
        ix = int(round(cx))
        iy = int(round((self.H - 1) - cy))
        return ix, iy

    def grid_to_world(self, ix, iy):
        cx = float(ix); cy = float((self.H - 1) - iy)
        return self.ox + cx*self.res, self.oy + cy*self.res

    def _free(self, ix, iy):
        return (0 <= ix < self.W) and (0 <= iy < self.H) and (not self.occ[iy, ix])

    # ---------- start pose ----------
    def get_start_pose(self):
        wait_s = float(self.get_parameter('tf_wait_s').get_parameter_value().double_value)
        deadline = self.get_clock().now() + Duration(seconds=wait_s)

        # 1) map->base_link
        while self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                return float(t.x), float(t.y), 'tf_map_base_link'
            except (LookupException, ConnectivityException, ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.05)

        # 2) map->odom + /odom
        try:
            tf_mo = self.tf_buffer.lookup_transform('map', 'odom', Time(), timeout=Duration(seconds=0.2))
            if self.last_odom is not None:
                ox = self.last_odom.pose.pose.position.x
                oy = self.last_odom.pose.pose.position.y
                tx = tf_mo.transform.translation
                return float(tx.x + ox), float(tx.y + oy), 'tf_map_odom_plus_odom'
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        # 3) /pose in map
        if self.last_pose_map is not None:
            p = self.last_pose_map.pose.position
            return float(p.x), float(p.y), 'pose_topic_map'

        # 4) params
        sx = float(self.get_parameter('start_x').get_parameter_value().double_value)
        sy = float(self.get_parameter('start_y').get_parameter_value().double_value)
        return sx, sy, 'params'

    # ---------- A* ----------
    def _astar(self, sx, sy, gx, gy):
        start = self.world_to_grid(sx, sy)
        goal  = self.world_to_grid(gx, gy)
        if not self._free(*start):
            nf = self._nearest_free(start)
            if nf is None: return None
            start = nf
        if not self._free(*goal):
            nf = self._nearest_free(goal)
            if nf is None: return None
            goal = nf

        sx_i, sy_i = start; gx_i, gy_i = goal
        g = np.full((self.H, self.W), np.inf, np.float32)
        came = np.full((self.H, self.W, 2), -1, np.int32)
        g[sy_i, sx_i] = 0.0

        nbrs = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
                (-1,-1,math.sqrt(2)),(1,-1,math.sqrt(2)),
                (-1,1,math.sqrt(2)),(1,1,math.sqrt(2))]

        def h(x,y):
            dx,dy=abs(x-gx_i),abs(y-gy_i)
            return (dx+dy)+(math.sqrt(2)-2)*min(dx,dy)

        openq = [(h(sx_i,sy_i),0,sx_i,sy_i)]
        closed = np.zeros((self.H,self.W),bool)

        while openq:
            _,_,x,y = heapq.heappop(openq)
            if closed[y,x]: continue
            closed[y,x]=True
            if (x,y)==(gx_i,gy_i):
                path=[]; cx,cy=x,y
                while (cx,cy)!=(sx_i,sy_i):
                    path.append((cx,cy))
                    px,py=came[cy,cx]
                    if px < 0: break
                    cx,cy=px,py
                path.append((sx_i,sy_i))
                path.reverse(); return path
            for dx,dy,w in nbrs:
                nx,ny=x+dx,y+dy
                if not self._free(nx,ny): continue
                if closed[ny,nx]: continue
                # prevent diagonal corner cutting
                if dx!=0 and dy!=0:
                    if not ( self._free(x+dx,y) and self._free(x,y+dy) ):
                        continue
                ng=g[y,x]+w
                if ng<g[ny,nx]:
                    g[ny,nx]=ng; came[ny,nx]=(x,y)
                    heapq.heappush(openq,(ng+h(nx,ny),ng,nx,ny))
        return None

    def _nearest_free(self, cell, max_r=30):
        x0, y0 = cell
        if self._free(x0, y0): return (x0, y0)
        for r in range(1, max_r+1):
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    x, y = x0+dx, y0+dy
                    if self._free(x, y):
                        return (x, y)
        return None

    # ---------- main ----------
    def plan_and_publish(self):
        sx, sy, src = self.get_start_pose()
        self.get_logger().info(f"Start source: {src} at ({sx:.2f},{sy:.2f})")

        gx = float(self.get_parameter('goal_x').get_parameter_value().double_value)
        gy = float(self.get_parameter('goal_y').get_parameter_value().double_value)

        t0 = time.time()
        path_cells = self._astar(sx, sy, gx, gy)
        dt = (time.time()-t0)*1000.0

        if path_cells is None:
            self.get_logger().error("A* failed (no path).")
            self._save_overlay_png(None, sx, sy, gx, gy)
            return

        # nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = now_msg(self)

        length = 0.0
        last = None
        for ix, iy in path_cells:
            wx, wy = self.grid_to_world(ix, iy)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            if last is not None:
                length += math.hypot(wx-last[0], wy-last[1])
            last = (wx, wy)
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self._save_path_files(path_cells)
        self.get_logger().info(f"A* OK: waypoints={len(path_msg.poses)} length={length:.2f} m  time={dt:.1f} ms")
        self._save_overlay_png(path_cells, sx, sy, gx, gy)

    # ---------- overlay (outside-only margin line) ----------
    def _save_overlay_png(self, path_cells, sx, sy, gx, gy):
        """Overlay:
        - Magenta: outside edge on KNOWN-FREE pixels adjacent to inflated no-go
        - Red: path
        - Green/Blue: start/goal
        """
        if not _HAVE_MPL:
            self.get_logger().warn("matplotlib not available; skipping overlay.")
            return

        img = np.stack([self.img_gray]*3, axis=-1)

        occ_inf = self.occ_inflated              # True = inflated/blocked
        unk     = self.unknown_mask_base         # True = unknown (mid-gray band)

        # Known-free pixels (not inflated, not unknown)
        known_free = (~occ_inf) & (~unk)

        # 4-neighbor adjacency to inflated cells
        up    = np.zeros_like(occ_inf, dtype=bool); up[1:,  :] = occ_inf[:-1, :]
        down  = np.zeros_like(occ_inf, dtype=bool); down[:-1,:] = occ_inf[1:,  :]
        left_ = np.zeros_like(occ_inf, dtype=bool); left_[:,1:] = occ_inf[:, :-1]
        right = np.zeros_like(occ_inf, dtype=bool); right[:, :-1] = occ_inf[:, 1:]

        # Margin ONLY on known-free pixels that touch inflated
        outside_margin = known_free & (up | down | left_ | right)

        # (Optional) one-pixel erosion to avoid outer map frame artifacts:
        # outside_margin[[0,-1], :] = False
        # outside_margin[:, [0,-1]] = False

        my, mx = np.where(outside_margin)
        img[my, mx, 0] = 128
        img[my, mx, 1] = 128
        img[my, mx, 2] = 128

        # Path (red, slightly thick)
        if path_cells is not None:
            for (ix, iy) in path_cells:
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        x = ix + dx; y = iy + dy
                        if 0 <= x < self.W and 0 <= y < self.H:
                            img[y, x, 0] = 255
                            img[y, x, 1] = 0
                            img[y, x, 2] = 0

        # Start (green), Goal (blue)
        s_ix, s_iy = self.world_to_grid(sx, sy)
        g_ix, g_iy = self.world_to_grid(gx, gy)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                xs, ys = s_ix + dx, s_iy + dy
                xg, yg = g_ix + dx, g_iy + dy
                if 0 <= xs < self.W and 0 <= ys < self.H:
                    img[ys, xs, 0] = 0; img[ys, xs, 1] = 255; img[ys, xs, 2] = 0
                if 0 <= xg < self.W and 0 <= yg < self.H:
                    img[yg, xg, 0] = 0; img[yg, xg, 1] = 128; img[yg, xg, 2] = 255

        os.makedirs(self.save_dir, exist_ok=True)
        out_path = os.path.join(self.save_dir, f"{self.map_stem}_astar_overlay.png")
        try:
            plt.figure(figsize=(10, 5))
            plt.imshow(img); plt.axis('off'); plt.tight_layout(pad=0)
            plt.savefig(out_path, dpi=150, bbox_inches='tight', pad_inches=0)
            plt.close()
            self.get_logger().info(f"Saved overlay: {out_path}")
        except Exception as e:
            self.get_logger().warn(f"Overlay save failed: {e}")



    # ---------- saving path files ----------
    def _poses_cells_to_world(self, path_cells):
        pts = []
        for ix, iy in path_cells:
            wx, wy = self.grid_to_world(ix, iy)
            pts.append((wx, wy))
        return pts

    def _save_path_files(self, path_cells):
        if not self.get_parameter('save_path').get_parameter_value().bool_value:
            return
        fmt = self.get_parameter('save_format').get_parameter_value().string_value.lower()
        base = self.get_parameter('save_basename').get_parameter_value().string_value
        if not base:
            base = f"{self.map_stem}_astar_path"
        world_pts = self._poses_cells_to_world(path_cells)

        os.makedirs(self.save_dir, exist_ok=True)

        if fmt in ('csv', 'both'):
            csv_path = os.path.join(self.save_dir, base + '.csv')
            try:
                with open(csv_path, 'w') as f:
                    f.write("# x[m],y[m]\n")
                    for x, y in world_pts:
                        f.write(f"{x:.6f},{y:.6f}\n")
                self.get_logger().info(f"Saved path CSV: {csv_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to save CSV: {e}")

        if fmt in ('yaml', 'both'):
            yaml_path = os.path.join(self.save_dir, base + '.yaml')
            try:
                data = {
                    'frame_id': 'map',
                    'resolution_m': float(self.res),
                    'origin_xy': [float(self.ox), float(self.oy)],
                    'waypoints': [{'x': float(x), 'y': float(y)} for (x, y) in world_pts]
                }
                try:
                    import yaml  # type: ignore
                    with open(yaml_path, 'w') as f:
                        yaml.safe_dump(data, f, sort_keys=False)
                except Exception:
                    with open(yaml_path, 'w') as f:
                        f.write(f"frame_id: map\nresolution_m: {self.res}\norigin_xy: [{self.ox}, {self.oy}]\nwaypoints:\n")
                        for x, y in world_pts:
                            f.write(f"  - {{x: {x}, y: {y}}}\n")
                self.get_logger().info(f"Saved path YAML: {yaml_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to save YAML: {e}")

    # ---------- shutdown ----------
    def _shutdown(self):
        self.get_logger().info("Shutting down.")
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
