"""Target source: a depth camera, transformed into the machine's base frame.

This is the implementation where units actually bite, so it is written the way
the lesson says it should be:

1. The driver hands us millimetres in the camera optical frame.
2. We convert to metres **immediately**, before stamping a frame onto it.
3. Only then does the point go anywhere near TF.

Doing step 2 late — or not at all — is how a millimetre-valued point ends up
in ``do_transform_point`` beside a metre-valued transform. The rotation still
looks right, the translation is off by a factor of a thousand, and nothing in
the stack complains. See ``units.py``.

Serves: ``get_target`` (plc_seam_interfaces/GetTarget)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point)
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from plc_seam_interfaces.srv import GetTarget
from plc_seam.units import point_mm_to_m


class CameraTargetSource(Node):

    def __init__(self, detector=None):
        super().__init__('target_source_camera')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')

        # Injected so the node is testable without hardware. Must return
        # (x_mm, y_mm, z_mm) in the camera optical frame, or None.
        self._detector = detector

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._srv = self.create_service(GetTarget, 'get_target', self.on_request)
        self.get_logger().info('Camera target source ready on get_target')

    def on_request(self, request, response):
        reading_mm = self._detector() if self._detector else None
        if reading_mm is None:
            self.get_logger().warn('No valid detection this cycle')
            return response

        in_camera = PointStamped()
        in_camera.header.stamp = self.get_clock().now().to_msg()
        in_camera.header.frame_id = self.get_parameter('camera_frame').value
        in_camera.point.x, in_camera.point.y, in_camera.point.z = reading_mm

        # ---- the boundary. Millimetres stop here. ----
        point_mm_to_m(in_camera.point)

        base_frame = self.get_parameter('base_frame').value
        try:
            transform = self._tf_buffer.lookup_transform(
                base_frame, in_camera.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().error(f'Could not transform target: {exc}')
            return response

        response.target = tf2_geometry_msgs.do_transform_point(in_camera, transform)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy already shut the context down
    finally:
        rclpy.try_shutdown()
