#!/usr/bin/env python3

import os
import re
import threading

import rclpy

from collections import OrderedDict
from rclpy.exceptions import ROSInterruptException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray

from fbot_world import marker_utils, yaml_io

'''
Interactively annotate room (and object sub-area) boundaries by clicking points
in RViz, then write them to a YAML file under the 'rooms' section in the same
format consumed by pose.py (PosePlugin).

Workflow: launch navigation with the map and RViz, run this node, then use the
RViz "Publish Point" tool to click the corners of each room. The in-progress and
completed polygons are republished live as markers so you can see the boundaries
as you draw them. Only the 'rooms' section is written; the 'poses' section (filled
by pose_writer.py / place_pose_writer.py) is preserved untouched.

Created by Gabriel Dorneles on 2026-06-08.
'''


# Latched so RViz still receives the latest markers if it subscribes late.
qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

NAME_RE = re.compile(r"[A-Za-z0-9_.-]+")

# Same palettes as PosePlugin so the preview matches the runtime markers.
ROOM_COLORS = [
    (0.2, 0.6, 1.0, 0.35),   # blue
    (0.2, 1.0, 0.4, 0.35),   # green
    (1.0, 0.6, 0.1, 0.35),   # orange
    (0.9, 0.2, 0.9, 0.35),   # magenta
    (0.1, 0.9, 0.9, 0.35),   # cyan
    (1.0, 1.0, 0.1, 0.35),   # yellow
]

OBJECT_COLORS = [
    (1.0, 0.2, 0.2, 0.45),   # red
    (1.0, 0.8, 0.2, 0.45),   # amber
    (0.6, 0.2, 1.0, 0.45),   # purple
    (0.2, 1.0, 0.8, 0.45),   # teal
]

ACTIVE_COLOR = (1.0, 1.0, 1.0, 0.9)


class RoomWriter(Node):
    '''
    @class RoomWriter
    @brief A ROS 2 node that captures room/object boundary polygons from RViz
    "Publish Point" clicks and saves them to a YAML file's 'rooms' section.
    '''

    def __init__(self):
        '''
        @brief Constructor for the RoomWriter node.
        '''
        super().__init__(node_name='room_writer')

        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('marker_topic', '/room_writer/preview')
        self.declare_parameter('frame', 'map')

        self.clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame = self.get_parameter('frame').get_parameter_value().string_value

        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../.."))
        self.config_path = os.path.join(ws_dir, "src", "fbot_world", "fbot_world", "config")

        while True:
            self.yaml_file = input("Enter the name of the file to save the rooms (e.g., 'pose_finals.yaml'): ").strip()
            if self.yaml_file.endswith('.yaml'):
                break
            self.get_logger().warning("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = os.path.join(self.config_path, self.yaml_file)

        # State shared with the clicked-point callback (which runs on the spin
        # thread while the main thread blocks on input()).
        self.lock = threading.Lock()
        self.current_points = []
        self.active_label = None
        self.completed_rooms = OrderedDict()

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.click_sub = self.create_subscription(
            PointStamped, self.clicked_topic, self.clicked_cb, 10
        )

        # Spin in the background so clicked points are received while the main
        # thread blocks on input().
        self._stop_event = threading.Event()
        self.executor_thread = threading.Thread(target=self._spin, daemon=True)
        self.executor_thread.start()

    def _spin(self) -> None:
        # spin_once in a loop (rather than rclpy.spin) so stop() can end the
        # thread before the node is destroyed, avoiding a teardown race.
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
        except (ROSInterruptException, rclpy.executors.ExternalShutdownException):
            pass

    def stop(self) -> None:
        '''@brief Stop the background spin thread and wait for it to finish.'''
        self._stop_event.set()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Clicked-point handling and live preview
    # ------------------------------------------------------------------

    def clicked_cb(self, msg: PointStamped) -> None:
        '''@brief Append a clicked point to the active polygon and refresh preview.'''
        point = [msg.point.x, msg.point.y]
        with self.lock:
            self.current_points.append(point)
            count = len(self.current_points)
        self.get_logger().info(f"Point {count}: ({point[0]:.3f}, {point[1]:.3f})")
        self.publish_preview()

    def publish_preview(self) -> None:
        '''
        @brief Rebuild and publish the full marker array: every completed room and
        object, plus the in-progress polygon. A leading DELETEALL clears stale
        markers so undo/reset are reflected.
        '''
        with self.lock:
            rooms = [(name, data) for name, data in self.completed_rooms.items()]
            active = [list(p) for p in self.current_points]

        marker_array = MarkerArray()
        marker_array.markers.append(marker_utils.delete_all_marker(self.frame))
        marker_id = 0

        for room_idx, (room_name, room_data) in enumerate(rooms):
            room_color = ROOM_COLORS[room_idx % len(ROOM_COLORS)]
            vertices = room_data.get('vertices', [])
            if vertices:
                marker_array.markers.append(
                    marker_utils.polygon_fill(vertices, marker_id, 'rooms', room_color, z=0.01, frame=self.frame))
                marker_id += 1
                marker_array.markers.append(
                    marker_utils.polygon_line_strip(vertices, marker_id, 'rooms', room_color, z=0.02, line_width=0.06, frame=self.frame))
                marker_id += 1
                cx, cy = marker_utils.centroid(vertices)
                marker_array.markers.append(
                    marker_utils.text_marker(room_name, (cx, cy), marker_id, 'rooms', z=0.5, text_size=0.3, frame=self.frame))
                marker_id += 1

            objects = room_data.get('objects', {})
            for obj_idx, (obj_name, obj_vertices) in enumerate(objects.items()):
                if not obj_vertices:
                    continue
                obj_color = OBJECT_COLORS[obj_idx % len(OBJECT_COLORS)]
                ns = f'room_{room_name}_objects'
                marker_array.markers.append(
                    marker_utils.polygon_fill(obj_vertices, marker_id, ns, obj_color, z=0.03, frame=self.frame))
                marker_id += 1
                marker_array.markers.append(
                    marker_utils.polygon_line_strip(obj_vertices, marker_id, ns, obj_color, z=0.04, line_width=0.04, frame=self.frame))
                marker_id += 1
                cx, cy = marker_utils.centroid(obj_vertices)
                marker_array.markers.append(
                    marker_utils.text_marker(f"{room_name}\n{obj_name}", (cx, cy), marker_id, ns, z=0.25, text_size=0.18, frame=self.frame))
                marker_id += 1

        # In-progress polygon: spheres at each click plus an open outline.
        if active:
            marker_array.markers.append(
                marker_utils.point_spheres(active, marker_id, 'active', ACTIVE_COLOR, scale=0.12, z=0.06, frame=self.frame))
            marker_id += 1
            if len(active) >= 2:
                marker_array.markers.append(
                    marker_utils.polygon_line_strip(active, marker_id, 'active', ACTIVE_COLOR, z=0.06, line_width=0.04, frame=self.frame, closed=False))
                marker_id += 1

        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Interactive capture
    # ------------------------------------------------------------------

    def capture_polygon(self, label: str) -> list:
        '''
        @brief Capture one polygon: the user clicks corners in RViz, then commands
        the terminal to close it.
        @return The list of [x, y] vertices, or None if aborted.
        '''
        with self.lock:
            self.current_points = []
            self.active_label = label
        self.publish_preview()

        print(f"\n  Drawing '{label}'. Use the RViz 'Publish Point' tool to click each corner.")
        while True:
            cmd = input("  [Enter]=close polygon  u=undo last  r=reset  a=abort : ").strip().lower()
            if cmd == '':
                with self.lock:
                    count = len(self.current_points)
                if count < 3:
                    self.get_logger().warning(f"A polygon needs at least 3 points (have {count}).")
                    continue
                with self.lock:
                    points = [list(p) for p in self.current_points]
                    self.current_points = []
                    self.active_label = None
                # Don't publish here: the caller registers the closed polygon
                # (as a room or object) and then refreshes the preview, so the
                # finished shape is drawn instead of a momentary blank.
                return points
            elif cmd == 'u':
                with self.lock:
                    if self.current_points:
                        removed = self.current_points.pop()
                        self.get_logger().info(f"Removed point ({removed[0]:.3f}, {removed[1]:.3f}).")
                self.publish_preview()
            elif cmd == 'r':
                with self.lock:
                    self.current_points = []
                self.get_logger().info("Reset current polygon.")
                self.publish_preview()
            elif cmd == 'a':
                with self.lock:
                    self.current_points = []
                    self.active_label = None
                self.publish_preview()
                return None
            else:
                self.get_logger().warning("Unknown command. Use Enter, u, r or a.")

    def _ask_name(self, prompt: str) -> str:
        '''@brief Read a name and validate it against the allowed character set.'''
        name = input(prompt).strip()
        if name == '':
            return ''
        if not NAME_RE.fullmatch(name):
            self.get_logger().warning("Invalid name. Use only letters, numbers, '-', '_' or '.'.")
            return None
        return name

    def run(self) -> None:
        '''@brief Main interactive session: capture rooms until the user is done.'''
        print("\n=== Room annotator ===")
        print("Make sure RViz has a MarkerArray display on topic "
              f"'{self.marker_topic}' (frame '{self.frame}').")
        self.publish_preview()

        while rclpy.ok():
            room_name = self._ask_name("\nRoom name (blank to finish and save): ")
            if room_name is None:
                continue
            if room_name == '':
                break
            if room_name in self.completed_rooms:
                overwrite = input(f"Room '{room_name}' already exists. Overwrite? (y/n): ").strip().lower()
                if overwrite != 'y':
                    continue

            vertices = self.capture_polygon(f"room:{room_name}")
            if vertices is None:
                self.get_logger().info(f"Aborted room '{room_name}'.")
                continue

            # Register the room as soon as it's closed, so the finished polygon
            # (filled, labelled with its name, coloured) is shown immediately and
            # stays visible while we prompt for pose names and draw its objects.
            room_entry = OrderedDict()
            room_entry['poses'] = []
            room_entry['vertices'] = vertices
            room_entry['objects'] = OrderedDict()
            with self.lock:
                self.completed_rooms[room_name] = room_entry
            self.publish_preview()

            poses_raw = input("  Pose names in this room (comma-separated, blank for none): ")
            room_entry['poses'] = [p.strip() for p in poses_raw.split(',') if p.strip()]

            while True:
                obj_name = self._ask_name("  Object sub-area name (blank to finish this room): ")
                if obj_name is None:
                    continue
                if obj_name == '':
                    break
                obj_vertices = self.capture_polygon(f"{room_name}/{obj_name}")
                if obj_vertices is None:
                    self.get_logger().info(f"Aborted object '{obj_name}'.")
                    continue
                with self.lock:
                    room_entry['objects'][obj_name] = obj_vertices
                self.publish_preview()

            self.get_logger().info(
                f"Room '{room_name}' captured: {len(vertices)} vertices, "
                f"{len(room_entry['objects'])} object(s), {len(room_entry['poses'])} pose name(s)."
            )

        if self.completed_rooms:
            self.write_to_yaml()
            self.get_logger().info(f"Rooms saved to {self.yaml_file}. Shutting down node.")
        else:
            self.get_logger().info("No rooms captured. Nothing written.")

    def write_to_yaml(self) -> None:
        '''
        @brief Merge the captured rooms into the YAML file's 'rooms' section,
        preserving the existing 'poses' section and any other top-level keys.
        '''
        if os.path.exists(self.yaml_path):
            self.get_logger().info(f"{self.yaml_file} already exists. Rooms will be merged into the existing data.")
        else:
            self.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")

        existing_data = yaml_io.load_ordered(self.yaml_path)

        rooms = existing_data.get('rooms')
        if not isinstance(rooms, (dict, OrderedDict)):
            rooms = OrderedDict()
        rooms.update(self.completed_rooms)
        existing_data['rooms'] = rooms

        yaml_io.dump_ordered(existing_data, self.yaml_path)


def main(args=None) -> None:
    writer = None
    try:
        rclpy.init(args=args)
        writer = RoomWriter()
        writer.run()
    except (ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        if writer is not None:
            writer.stop()
            writer.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
