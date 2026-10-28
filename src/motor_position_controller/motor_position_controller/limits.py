"""Read actuator command bounds from the robot description."""

import math
from xml.etree import ElementTree

JOINT_NAMES = ('joint_motor_1_rotor', 'joint_motor_2_rotor')


def read_limits(description):
    """Return finite lower/upper bounds for both motor joints, in radians."""
    robot = ElementTree.fromstring(description)
    result = []
    for name in JOINT_NAMES:
        joints = [joint for joint in robot.findall('joint') if joint.get('name') == name]
        if len(joints) != 1 or joints[0].get('type') != 'revolute':
            raise ValueError(f'{name}: expected one bounded revolute joint')
        limit = joints[0].find('limit')
        if limit is None:
            raise ValueError(f'{name}: missing limits')
        lower, upper = float(limit.attrib['lower']), float(limit.attrib['upper'])
        if not (math.isfinite(lower) and math.isfinite(upper) and lower <= 0 <= upper
                and lower < upper):
            raise ValueError(f'{name}: invalid limits or zero outside bounds')
        result.append((lower, upper))
    return tuple(result)


def clamp_position(value, limits):
    """Reject nonfinite commands and constrain a position to one joint's limits."""
    if not math.isfinite(value):
        raise ValueError('position must be finite')
    return max(limits[0], min(float(value), limits[1]))
