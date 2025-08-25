#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np

class MapVisualizerNode(Node):
    def __init__(self):
        super().__init__('map_visualizer_node')

        # Param (can be left blank)
        self.declare_parameter('map_yaml_path', '')
        yaml_path = (self.get_parameter('map_yaml_path').get_parameter_value().string_value or '').strip()

        # Default to <ws>/src/autonomous_explorer/map/my_map1.yaml if empty
        if not yaml_path:
            yaml_path = self._default_map_yaml_path()

        # Expand ~ and $VARS and accept relative paths
        yaml_path = os.path.abspath(os.path.expanduser(os.path.expandvars(yaml_path)))

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"❌ Map YAML path does not exist: {yaml_path}")
            rclpy.get_default_context().shutdown()
            return

        # Load map and metadata
        if not self.load_map(yaml_path):
            rclpy.get_default_context().shutdown()
            return

        # Publisher with TRANSIENT_LOCAL so late subscribers (AMCL/RViz) get the last map
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)

        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_map)
        self._has_logged = False

    def _default_map_yaml_path(self) -> str:
        """
        Resolve default to: <workspace>/src/autonomous_explorer/map/my_map1.yaml.
        1) Try backtracking from ament share dir (robust when installed).
        2) Fallback to CWD-based guess (works if running from workspace root).
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('autonomous_explorer')
            # share_dir: <ws>/install/autonomous_explorer/share/autonomous_explorer
            install_pkg = os.path.dirname(os.path.dirname(share_dir))  # .../install/autonomous_explorer
            install_root = os.path.dirname(install_pkg)                # .../install
            ws_root = os.path.dirname(install_root)                    # .../<ws>
            candidate = os.path.join(ws_root, 'src', 'autonomous_explorer', 'map', 'my_map1.yaml')
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

        # Fallback guess from current working directory
        return os.path.abspath(os.path.join(os.getcwd(), 'src', 'autonomous_explorer', 'map', 'my_map1.yaml'))

    def load_map(self, yaml_path: str) -> bool:
        try:
            with open(yaml_path, 'r') as f:
                metadata = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to read YAML: {yaml_path} ({e})")
            return False

        image_path = os.path.join(os.path.dirname(yaml_path), metadata['image'])
        try:
            resolution = float(metadata['resolution'])
            origin = metadata['origin']
        except Exception as e:
            self.get_logger().error(f"❌ Bad YAML format (missing keys?): {e}")
            return False

        if not os.path.exists(image_path):
            self.get_logger().error(f"❌ Map image file not found: {image_path}")
            return False

        self.get_logger().info(f"✅ Loaded map image: {image_path}")

        # Load grayscale
        try:
            img = Image.open(image_path).convert('L')
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open image: {e}")
            return False

        data = np.array(img)
        height, width = data.shape

        # Convert to occupancy grid (simple PGM policy)
        map_data = []
        for y in range(height):
            row = data[height - 1 - y]  # Flip vertically for ROS map convention
            for px in row:
                if px >= 254:
                    occ = 0      # free
                elif px <= 1:
                    occ = 100    # occupied
                else:
                    occ = -1     # unknown
                map_data.append(occ)

        # Fill OccupancyGrid
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        self.map_msg.info.height = height
        self.map_msg.info.origin.position.x = float(origin[0])
        self.map_msg.info.origin.position.y = float(origin[1])
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.x = 0.0
        self.map_msg.info.origin.orientation.y = 0.0
        self.map_msg.info.origin.orientation.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = map_data
        return True

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)
        if not self._has_logged:
            self.get_logger().info("🗺️ Publishing /map (TRANSIENT_LOCAL for AMCL and RViz)")
            self._has_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
