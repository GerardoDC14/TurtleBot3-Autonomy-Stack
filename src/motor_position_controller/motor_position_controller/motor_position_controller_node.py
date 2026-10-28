#!/usr/bin/env python3
"""Publish explicit operator commands bounded by the installed URDF."""

from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from motor_position_controller.limits import clamp_position, read_limits


class MotorPositionControllerNode(Node):
    """Publish references at 20 Hz after an operator enables each axis."""

    def __init__(self):
        super().__init__('motor_position_controller')
        model = Path(get_package_share_directory('motor_position_controller')) / 'models' / 'parallel.urdf'
        self.declare_parameter('model_file', str(model))
        self.declare_parameter('robot_description', '')
        self.declare_parameter('interactive', True)
        description = self.get_parameter('robot_description').value
        if not description:
            description = Path(self.get_parameter('model_file').value).read_text(encoding='utf-8')
        self.limits = read_limits(description)
        self.positions = [0.0, 0.0]
        self.enabled = [False, False]
        self.lock = threading.Lock()
        self.publishers_by_axis = [self.create_publisher(Float32, f'motor{i}_position', 10)
                                   for i in (1, 2)]
        self.timer = self.create_timer(0.05, self.timer_callback)
        if self.get_parameter('interactive').value:
            threading.Thread(target=self.keyboard_input_thread, daemon=True).start()

    def set_position(self, target, value):
        """Validate a complete command before changing either axis."""
        targets = {'both': (0, 1), 'motor1': (0,), 'motor2': (1,)}
        if target not in targets:
            raise ValueError('unknown axis')
        updates = {axis: clamp_position(value, self.limits[axis]) for axis in targets[target]}
        with self.lock:
            for axis, position in updates.items():
                self.positions[axis] = position
                self.enabled[axis] = True
        self.get_logger().info(f'Position reference in rad: {updates}')

    def timer_callback(self):
        """Publish a consistent snapshot; do not command untouched axes."""
        with self.lock:
            snapshot = list(zip(self.enabled, self.positions))
        for publisher, (enabled, position) in zip(self.publishers_by_axis, snapshot):
            if enabled:
                publisher.publish(Float32(data=position))

    def keyboard_input_thread(self):
        self.get_logger().info('Commands: T <rad>, T1 <rad>, T2 <rad>, q to quit.')
        while rclpy.ok():
            try:
                command = input().strip()
                if command == 'q':
                    rclpy.try_shutdown()
                    return
                prefix, raw_value = command.split(maxsplit=1)
                target = {'T': 'both', 'T1': 'motor1', 'T2': 'motor2'}.get(prefix)
                self.set_position(target, float(raw_value))
            except (EOFError, KeyboardInterrupt):
                rclpy.try_shutdown()
                return
            except (ValueError, TypeError) as exc:
                self.get_logger().warning(f'Invalid command: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorPositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
