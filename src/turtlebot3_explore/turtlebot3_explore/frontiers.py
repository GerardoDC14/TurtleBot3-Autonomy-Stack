"""Occupancy-grid geometry and frontier selection without ROS dependencies."""

import math

import numpy as np
from scipy.ndimage import convolve, distance_transform_edt, label


class Grid:
    """Cache geometry, candidate groups and unknown-cell sums for one map."""

    def __init__(self, message, free_threshold=25):
        self.width, self.height = int(message.info.width), int(message.info.height)
        self.resolution = float(message.info.resolution)
        self.origin_x = message.info.origin.position.x
        self.origin_y = message.info.origin.position.y
        q = message.info.origin.orientation
        values = (self.resolution, self.origin_x, self.origin_y, q.x, q.y, q.z, q.w)
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0
                or not all(math.isfinite(v) for v in values)
                or len(message.data) != self.width * self.height):
            raise ValueError('invalid occupancy-grid dimensions, resolution or origin')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9:
            raise ValueError('invalid map-origin quaternion')
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        if abs(x) > 1e-6 or abs(y) > 1e-6:
            raise ValueError('occupancy-grid origin must be planar')
        self.yaw = math.atan2(2*w*z, 1-2*z*z)
        self.cos, self.sin = math.cos(self.yaw), math.sin(self.yaw)
        self.data = np.asarray(message.data, dtype=np.int16).reshape(self.height, self.width)
        if np.any((self.data < -1) | (self.data > 100)):
            raise ValueError('occupancy values must be -1 or 0..100')
        self.free = (self.data >= 0) & (self.data <= free_threshold)
        self.unknown = self.data == -1
        traversable = ~((self.data > free_threshold) & ~self.unknown)
        self.clearance = distance_transform_edt(
            np.pad(traversable, 1, constant_values=False))[1:-1, 1:-1] * self.resolution
        self.integral = np.pad(self.unknown.astype(np.int64), ((1, 0), (1, 0))).cumsum(0).cumsum(1)

    def world(self, col, row):
        """Convert cell centers to world coordinates, including origin yaw."""
        x, y = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        return (self.origin_x + self.cos*x - self.sin*y,
                self.origin_y + self.sin*x + self.cos*y)

    def cell(self, x, y):
        """Use floor so points just outside the negative boundary stay outside."""
        dx, dy = x-self.origin_x, y-self.origin_y
        return (math.floor((self.cos*dx + self.sin*dy) / self.resolution),
                math.floor((-self.sin*dx + self.cos*dy) / self.resolution))

    def valid(self, coord, clearance=0.0):
        col, row = self.cell(*coord)
        return (0 <= col < self.width and 0 <= row < self.height
                and bool(self.free[row, col]) and self.clearance[row, col] >= clearance)

    def candidates(self, min_size=5, clearance=0.2):
        """Choose a real free member near each connected frontier's centroid."""
        neighbors = convolve(self.unknown.astype(np.int8), np.ones((3, 3), dtype=np.int8),
                             mode='constant', cval=0)
        frontier = self.free & (neighbors > 0)
        groups, count = label(frontier, structure=np.ones((3, 3), dtype=int))
        result = []
        for group in range(1, count + 1):
            cells = np.argwhere(groups == group)
            if len(cells) < min_size:
                continue
            safe = cells[self.clearance[cells[:, 0], cells[:, 1]] >= clearance]
            if len(safe) == 0:
                continue
            center = cells.mean(axis=0)
            row, col = safe[np.argmin(((safe-center)**2).sum(axis=1))]
            result.append(self.world(col, row))
        return result

    def information_gain(self, coord, radius):
        """Return unknown area in a square window, in square meters."""
        col, row = self.cell(*coord)
        size = math.ceil(radius / self.resolution)
        x0, x1 = max(0, col-size), min(self.width, col+size+1)
        y0, y1 = max(0, row-size), min(self.height, row+size+1)
        if x0 >= x1 or y0 >= y1:
            return 0.0
        a = self.integral
        count = a[y1, x1] - a[y0, x1] - a[y1, x0] + a[y0, x0]
        return float(count) * self.resolution**2
