"""Check map geometry, retry configuration and actuator bounds."""

import ast
import math
import re
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yaml

from support import ROOT, callbacks, map_message
from motor_position_controller.limits import clamp_position, read_limits
from turtlebot3_explore.frontiers import Grid


class GeometryTests(unittest.TestCase):
    def test_rotated_origin_round_trip(self):
        grid = Grid(map_message(np.zeros((4, 6), dtype=int), origin=(2.0, -3.0), yaw=math.pi/2))
        for row in range(grid.height):
            for col in range(grid.width):
                self.assertEqual(grid.cell(*grid.world(col, row)), (col, row))

    def test_negative_boundary_is_not_truncated_into_map(self):
        grid = Grid(map_message(np.zeros((4, 4), dtype=int)))
        self.assertFalse(grid.valid((-0.001, 0.1)))
        self.assertEqual(grid.cell(-0.001, 0.1)[0], -1)

    def test_frontier_target_is_a_free_member(self):
        data = np.full((12, 12), -1, dtype=int)
        data[3:9, 3:9] = 0
        data[5:7, 5:7] = 100
        grid = Grid(map_message(data))
        coords = grid.candidates(min_size=3, clearance=0.0)
        self.assertEqual(len(coords), 1)
        self.assertTrue(grid.valid(coords[0]))
        col, row = grid.cell(*coords[0])
        self.assertIn(-1, data[max(0, row-1):row+2, max(0, col-1):col+2])

    def test_clearance_excludes_obstacle_cells(self):
        data = np.zeros((7, 7), dtype=int)
        data[3, 3] = 100
        grid = Grid(map_message(data))
        self.assertFalse(grid.valid(grid.world(3, 3), clearance=0.0))
        self.assertFalse(grid.valid(grid.world(4, 3), clearance=0.3))
        self.assertTrue(grid.valid(grid.world(5, 3), clearance=0.3))

    def test_unknown_area_uses_cached_counts(self):
        data = np.full((5, 5), -1, dtype=int)
        data[2, 2] = 0
        grid = Grid(map_message(data))
        self.assertAlmostEqual(grid.information_gain(grid.world(2, 2), 0.2), 8*0.2**2)

    def test_malformed_maps_are_rejected(self):
        message = map_message(np.zeros((2, 2), dtype=int))
        message.data.pop()
        with self.assertRaises(ValueError):
            Grid(message)
        message = map_message(np.zeros((2, 2), dtype=int), resolution=0.0)
        with self.assertRaises(ValueError):
            Grid(message)

    def test_retry_limit_applies_before_first_success(self):
        cls = callbacks('src/turtlebot3_explore/turtlebot3_explore/explore.py',
                        {'math': math, 'time': NS(monotonic=lambda: 20.0)})['Explore']
        node = object.__new__(cls)
        node.max_attempts, node.frontier_visit_threshold = 3, 0.3
        node.visited_frontiers, node.retry_after = [], {}
        node.frontier_selection_attempts = {(1.0, 2.0): 3}
        self.assertEqual(node.filter_frontiers([(1.0, 2.0)]), [])
        node.frontier_selection_attempts[(1.0, 2.0)] = 1
        node.retry_after[(1.0, 2.0)] = 30.0
        self.assertEqual(node.filter_frontiers([(1.0, 2.0)]), [])

    def test_every_yaml_parameter_is_consumed_by_both_explorers(self):
        source = (ROOT / 'src/turtlebot3_explore/turtlebot3_explore/explore.py').read_text()
        tree = ast.parse(source)
        defaults = next(node.value for node in ast.walk(tree)
                        if isinstance(node, ast.Assign) and any(
                            isinstance(target, ast.Name) and target.id == 'defaults'
                            for target in node.targets))
        python_keys = set(ast.literal_eval(defaults))
        cpp = (ROOT / 'src/explore_cpp/src/explore_node.cpp').read_text()
        cpp_keys = set(re.findall(r'declare_parameter(?:<[^>]+>)?\("([^"]+)"', cpp))
        config = yaml.safe_load((ROOT / 'src/turtlebot3_explore/config/explore_params.yaml').read_text())
        for name in config['explore_node']['ros__parameters']:
            if name != 'use_sim_time':
                self.assertIn(name, python_keys)
                self.assertIn(name, cpp_keys)


class LimitTests(unittest.TestCase):
    def setUp(self):
        self.description = (ROOT / 'models/parallel.urdf').read_text()
        self.limits = read_limits(self.description)

    def test_commands_use_model_limits_for_each_axis(self):
        self.assertEqual(self.limits, ((-3.14, 3.14), (-3.14, 3.14)))
        self.assertEqual(clamp_position(20, self.limits[0]), 3.14)
        self.assertEqual(clamp_position(-20, self.limits[1]), -3.14)

    def test_nonfinite_commands_are_rejected(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                clamp_position(value, self.limits[0])

    def test_missing_limits_fail_closed(self):
        with self.assertRaises(ValueError):
            read_limits('<robot/>')
        with self.assertRaises(ValueError):
            read_limits(self.description.replace('upper="3.14"', 'upper="nan"'))
