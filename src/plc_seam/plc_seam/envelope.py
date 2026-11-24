"""The working window, read from ``config/envelope.yaml``.

The URDF joint limits, the PLC stand-in's window and the target sources'
ranges all come from that one file, so they cannot drift apart. Metres, in
``base_link``: the PLC stand-in converts its copy to millimetres itself.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

AXES = ('x', 'y', 'z')


def load(path=None):
    """Return {'x': (lower, upper), 'y': ..., 'z': ...} in metres."""
    if path is None:
        path = os.path.join(
            get_package_share_directory('plc_seam'), 'config', 'envelope.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return {axis: (float(data[axis][0]), float(data[axis][1])) for axis in AXES}
