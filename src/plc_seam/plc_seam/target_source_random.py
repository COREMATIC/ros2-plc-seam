"""Target source: pseudo-random points inside the working envelope.

This is the implementation you bring up first. It needs no camera, no
simulator and no machine — which means the PLC handshake can be exercised
end to end on a laptop.

Serves: ``get_target`` (plc_seam_interfaces/GetTarget)
"""

import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from plc_seam_interfaces.srv import GetTarget


class RandomTargetSource(Node):

    def __init__(self):
        super().__init__('target_source_random')

        # Envelope in METRES. Keep these inside the URDF joint limits.
        self.declare_parameter('x_range_m', [0.0, 0.75])
        self.declare_parameter('y_range_m', [0.0, 0.55])
        self.declare_parameter('z_range_m', [-1.02, -0.40])
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('targets_before_exhausted', 5)

        self._served = 0
        self._srv = self.create_service(GetTarget, 'get_target', self.on_request)
        self.get_logger().info('Random target source ready on get_target')

    def on_request(self, request, response):
        limit = self.get_parameter('targets_before_exhausted').value
        if limit > 0 and self._served >= limit:
            self._served = 0
            response.source_exhausted = True
            self.get_logger().info('Source exhausted; resetting')
            return response

        xr = self.get_parameter('x_range_m').value
        yr = self.get_parameter('y_range_m').value
        zr = self.get_parameter('z_range_m').value

        response.target.header.stamp = self.get_clock().now().to_msg()
        response.target.header.frame_id = self.get_parameter('frame_id').value
        response.target.point.x = random.uniform(*xr)
        response.target.point.y = random.uniform(*yr)
        response.target.point.z = random.uniform(*zr)
        response.success = True

        self._served += 1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomTargetSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy already shut the context down
    finally:
        rclpy.try_shutdown()
