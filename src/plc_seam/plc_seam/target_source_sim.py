"""Target source: whatever the simulator is currently looking at.

The simulator publishes a candidate target on a plain topic. This node caches
the latest one and re-serves it through the same service the other sources
use, so nothing downstream changes when you switch to it.

That indirection is the whole trick. The simulator was adopted here for about
a hundred lines precisely because this interface already existed for the
random source.

Subscribes: ``sim/target_point`` (geometry_msgs/PointStamped, METRES)
Serves:     ``get_target`` (plc_seam_interfaces/GetTarget)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from plc_seam_interfaces.srv import GetTarget


class SimTargetSource(Node):

    def __init__(self):
        super().__init__('target_source_sim')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('targets_before_exhausted', 5)
        # Floor for the vertical axis, in METRES. The simulator can report a
        # point below the reachable volume; clamp rather than command it.
        self.declare_parameter('z_min_m', -1.02)

        self._latest = None
        self._served = 0

        self.create_subscription(
            PointStamped, 'sim/target_point', self.on_sim_point, 10)
        self._srv = self.create_service(GetTarget, 'get_target', self.on_request)
        self.get_logger().info(
            'Simulator target source ready; waiting for sim/target_point')

    def on_sim_point(self, msg: PointStamped):
        z_min = self.get_parameter('z_min_m').value
        if msg.point.z < z_min:
            msg.point.z = z_min
        self._latest = msg

    def on_request(self, request, response):
        if self._latest is None:
            self.get_logger().warn('No simulator target received yet')
            return response

        limit = self.get_parameter('targets_before_exhausted').value
        if limit > 0 and self._served >= limit:
            self._served = 0
            response.source_exhausted = True
            return response

        response.target = self._latest
        response.target.header.stamp = self.get_clock().now().to_msg()
        if not response.target.header.frame_id:
            response.target.header.frame_id = self.get_parameter('frame_id').value
        response.success = True

        self._served += 1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimTargetSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy already shut the context down
    finally:
        rclpy.try_shutdown()
