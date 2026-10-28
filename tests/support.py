"""Load pure helpers and isolated callbacks without requiring a ROS install."""

import ast
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
for package in (ROOT / 'src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))


class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class PoseStamped:
    def __init__(self):
        self.header = NS(frame_id='', stamp=NS(sec=0, nanosec=0))
        self.pose = NS(position=Point(), orientation=NS(x=0.0, y=0.0, z=0.0, w=0.0))


class Marker:
    SPHERE, TEXT_VIEW_FACING, ADD, DELETEALL = 2, 9, 0, 3

    def __init__(self):
        pose = PoseStamped()
        self.header, self.pose = pose.header, pose.pose
        self.scale = Point()
        self.color = NS(r=0.0, g=0.0, b=0.0, a=0.0)
        self.ns, self.id, self.action, self.text = '', 0, 0, ''


class MarkerArray:
    def __init__(self):
        self.markers = []


def callbacks(relative, bindings=None):
    """Compile actual class/function bodies, replacing only ROS dependencies."""
    tree = ast.parse((ROOT / relative).read_text(encoding='utf-8'))
    selected = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))]
    scope = {'Node': object, 'Point': Point, 'PoseStamped': PoseStamped,
             'Marker': Marker, 'MarkerArray': MarkerArray, 'deepcopy': deepcopy}
    scope.update(bindings or {})
    exec(compile(ast.Module(body=selected, type_ignores=[]), relative, 'exec'), scope)
    return scope


def map_message(data, resolution=0.2, origin=(0.0, 0.0), yaw=0.0):
    import math
    return NS(data=data.ravel().tolist(), info=NS(
        width=data.shape[1], height=data.shape[0], resolution=resolution,
        origin=NS(position=Point(*origin), orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2)))))
