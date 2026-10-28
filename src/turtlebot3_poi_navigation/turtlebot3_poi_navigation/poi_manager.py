#!/usr/bin/env python3
"""Manage persistent planar POIs and one asynchronous Nav2 goal."""

from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile
import time
import tkinter as tk
from tkinter import messagebox, simpledialog

from autonomy_common.navigation import NavigationTask
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class POI:
    """A named map location; radius controls only its visual footprint."""

    def __init__(self, name, x, y, radius):
        self.name = str(name).strip()
        values = tuple(float(value) for value in (x, y, radius))
        if not self.name or not all(math.isfinite(value) for value in values) or values[2] <= 0:
            raise ValueError('name, finite coordinates and a positive radius are required')
        self.position = Point(x=values[0], y=values[1], z=0.0)
        self.radius = values[2]


class POIManager(Node):
    """Run ROS callbacks and Tk widgets together on the GUI's main thread."""

    def __init__(self):
        super().__init__('poi_manager')
        self.declare_parameter('poi_file', str(Path.home() / '.ros/turtlebot3_autonomy/pois.yaml'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('navigation_timeout', 120.0)
        self.poi_file = Path(self.get_parameter('poi_file').value).expanduser()
        self.map_frame = self.get_parameter('map_frame').value
        self.pois = {}
        self.root = None
        self.closing = False
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation = NavigationTask(
            self.action_client, NavigateToPose.Goal,
            self.get_parameter('navigation_timeout').value,
            self.navigation_finished, notify=self.get_logger().warning)
        self.marker_pub = self.create_publisher(MarkerArray, 'poi_markers', 10)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        self.watchdog = self.create_timer(0.2, self.navigation.tick)
        self.load_pois(self.poi_file)

    def load_pois(self, filepath):
        if not Path(filepath).exists():
            return
        try:
            data = yaml.safe_load(Path(filepath).read_text(encoding='utf-8')) or {}
            if not isinstance(data, dict):
                raise ValueError('POI file must contain a mapping')
            loaded = {}
            for name, attrs in data.items():
                poi = POI(name, attrs['x'], attrs['y'], attrs['radius'])
                if poi.name in loaded:
                    raise ValueError('duplicate POI name after trimming whitespace')
                loaded[poi.name] = poi
            self.pois = loaded
        except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Cannot load POIs: {exc}')

    def save_pois(self, filepath):
        """Replace storage atomically so interrupted writes leave valid YAML."""
        path = Path(filepath)
        temporary = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            data = {name: {'x': poi.position.x, 'y': poi.position.y, 'radius': poi.radius}
                    for name, poi in self.pois.items()}
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                             delete=False) as stream:
                temporary = stream.name
                yaml.safe_dump(data, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            return True
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Cannot save POIs: {exc}')
            return False
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def start_gui(self):
        self.root = tk.Tk()
        self.root.title('POI Manager')
        self.root.protocol('WM_DELETE_WINDOW', self.close_gui)
        self.poi_listbox = tk.Listbox(self.root, height=10, width=50)
        self.poi_listbox.pack()
        buttons = tk.Frame(self.root)
        buttons.pack()
        for text, command in (
                ('Add POI', self.add_poi_gui), ('Delete POI', self.delete_poi_gui),
                ('Navigate', self.navigate_to_poi_gui), ('Cancel', self.navigation.cancel)):
            tk.Button(buttons, text=text, command=command).pack(side=tk.LEFT)
        self.status = tk.StringVar(value='Ready')
        tk.Label(self.root, textvariable=self.status).pack()
        self.update_poi_listbox()
        self.root.after(50, self.poll_ros)
        self.root.mainloop()

    def poll_ros(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        if self.closing:
            if not self.navigation.busy or time.monotonic() >= self.close_deadline:
                if self.navigation.busy:
                    self.get_logger().error('Cancellation unconfirmed; verify the base is stopped')
                self.root.destroy()
                return
        self.root.after(50, self.poll_ros)

    def close_gui(self):
        self.closing = True
        self.close_deadline = time.monotonic() + 3.0
        self.navigation.cancel()
        self.status.set('Closing; waiting for cancellation')

    def update_poi_listbox(self):
        self.poi_listbox.delete(0, tk.END)
        for name in self.pois:
            self.poi_listbox.insert(tk.END, name)

    def add_poi_gui(self):
        name = simpledialog.askstring('Add POI', 'Name:', parent=self.root)
        if name is None:
            return
        fields = []
        for field in ('X [m]', 'Y [m]', 'Visualization radius [m]'):
            value = simpledialog.askfloat('Add POI', field, parent=self.root)
            if value is None:
                return
            fields.append(value)
        try:
            poi = POI(name, *fields)
            if poi.name in self.pois:
                raise ValueError('a POI with this name already exists')
        except ValueError as exc:
            messagebox.showerror('Invalid POI', str(exc), parent=self.root)
            return
        self.pois[poi.name] = poi
        if not self.save_pois(self.poi_file):
            del self.pois[poi.name]
            messagebox.showerror('Storage error', 'POI was not saved', parent=self.root)
        self.update_poi_listbox()

    def selected_name(self):
        selected = self.poi_listbox.curselection()
        return self.poi_listbox.get(selected[0]) if selected else None

    def delete_poi_gui(self):
        name = self.selected_name()
        if name and messagebox.askyesno('Delete POI', f'Delete {name}?', parent=self.root):
            old = self.pois.pop(name)
            if not self.save_pois(self.poi_file):
                self.pois[name] = old
                messagebox.showerror('Storage error', 'POI was not deleted', parent=self.root)
            self.update_poi_listbox()

    def navigate_to_poi_gui(self):
        name = self.selected_name()
        if name:
            self.navigate_to_poi(name)

    def navigate_to_poi(self, name):
        """Keep the current goal until it finishes or cancellation is confirmed."""
        if self.closing or name not in self.pois or self.navigation.busy:
            return False
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = deepcopy(self.pois[name].position)
        pose.pose.orientation.w = 1.0
        accepted = self.navigation.begin(pose)
        if self.root:
            self.status.set(f'Navigating to {name}' if accepted else 'Nav2 is unavailable')
        return accepted

    def navigation_finished(self, outcome):
        self.get_logger().info(f'Navigation {outcome.status}: {outcome.detail}')
        if self.root and not self.closing:
            self.status.set(f'{outcome.status}: {outcome.detail}')

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, (name, poi) in enumerate(self.pois.items()):
            for namespace, marker_type in (('poi_markers', Marker.SPHERE),
                                           ('poi_labels', Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = namespace, index
                marker.type, marker.action = marker_type, Marker.ADD
                marker.pose.position = deepcopy(poi.position)
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                if marker_type == Marker.SPHERE:
                    marker.scale.x = marker.scale.y = poi.radius * 2.0
                    marker.scale.z, marker.color.g = 0.1, 1.0
                else:
                    marker.pose.position.z += 0.2
                    marker.scale.z = 0.3
                    marker.color.r = marker.color.g = marker.color.b = 1.0
                    marker.text = name
                markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = POIManager()
    try:
        node.start_gui()
    except KeyboardInterrupt:
        node.navigation.cancel()
    finally:
        node.closing = True
        node.navigation.cancel()
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and node.navigation.busy and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.navigation.busy:
            node.get_logger().error('Cancellation unconfirmed; verify the base is stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
