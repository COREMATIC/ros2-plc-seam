"""The control side: a machine-initiated pick cycle.

Most ROS systems assume ROS decides when the robot moves. Here it does not.
The PLC owns motion and safety, and it asks ROS a question when it is ready
for an answer. That inverts the control flow, and the node below is what that
inversion looks like in rclpy.

The cycle
---------
1. PLC raises ``request_target``.
2. We see the RISING EDGE in a GPIO state callback and call ``get_target``
   asynchronously. One edge, one request — level-triggering here would flood
   the source.
3. The response is converted to machine units and published to the position
   command interfaces.
4. We raise ``target_ready``. The PLC executes the move and drops
   ``request_target``.
5. If the source is exhausted we raise ``source_empty`` instead, and clear it
   only once the PLC acknowledges with ``empty_ack``. Two-way, because a
   dropped boolean here means a gripper closing on nothing.

Everything is ``call_async`` with a done-callback. Blocking the executor in
this path would stall the very GPIO stream that drives it.

Subscribes: /gpio_command/gpio_states  (control_msgs/DynamicInterfaceGroupValues)
Publishes:  /gpio_command/commands     (control_msgs/DynamicInterfaceGroupValues)
            /forward_command/commands  (std_msgs/Float64MultiArray, MACHINE UNITS)
Calls:      get_target                 (plc_seam_interfaces/GetTarget)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue

from plc_seam_interfaces.srv import GetTarget
from plc_seam.handshake import (
    EMPTY_ACK, GPIO_COMMANDS, GPIO_STATES, REQUEST_TARGET, SETPOINT_COMMANDS,
    SOURCE_EMPTY, TARGET_READY)
from plc_seam.units import xyz_m_to_mm


class PlcBridge(Node):

    def __init__(self):
        super().__init__('plc_bridge')

        self._prev_request = 0.0
        self._in_flight = False
        self._source_empty = False          # we raised source_empty, no ack yet

        self._cmd_pub = self.create_publisher(
            Float64MultiArray, SETPOINT_COMMANDS, 10)
        self._gpio_pub = self.create_publisher(
            DynamicInterfaceGroupValues, GPIO_COMMANDS, 10)
        self.create_subscription(
            DynamicInterfaceGroupValues, GPIO_STATES, self.on_gpio_states, 10)

        self._client = self.create_client(GetTarget, 'get_target')
        self.get_logger().info('PLC bridge ready; waiting for machine requests')

    # ---- GPIO helpers ----------------------------------------------------

    @staticmethod
    def _read_gpio(msg: DynamicInterfaceGroupValues, name: str):
        """Return a GPIO value by NAME. Index order is not part of the contract."""
        try:
            i = msg.interface_groups.index(name)
        except ValueError:
            return None
        return msg.interface_values[i].values[0]

    def _set_gpio(self, name: str, value: float):
        msg = DynamicInterfaceGroupValues()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = [name]
        iv = InterfaceValue()
        iv.interface_names = ['command']
        iv.values = [value]
        msg.interface_values = [iv]
        self._gpio_pub.publish(msg)

    # ---- the cycle -------------------------------------------------------

    def on_gpio_states(self, msg: DynamicInterfaceGroupValues):
        # Booleans cross the ros2_control boundary as float64. Comparing
        # against 1.0 looks wrong and is correct.
        request = self._read_gpio(msg, REQUEST_TARGET)
        if request is None:
            return

        if request == 1.0 and self._prev_request != 1.0 and not self._in_flight:
            self.request_target()
        elif request != 1.0 and self._prev_request == 1.0:
            # The machine finished the move; drop our ready flag.
            self._set_gpio(TARGET_READY, 0.0)

        self._prev_request = request

        if self._source_empty and self._read_gpio(msg, EMPTY_ACK) == 1.0:
            self._source_empty = False
            self._set_gpio(SOURCE_EMPTY, 0.0)

    def request_target(self):
        if not self._client.service_is_ready():
            self.get_logger().warn('No target source available; skipping cycle')
            return
        self._in_flight = True
        self._client.call_async(GetTarget.Request()).add_done_callback(
            self.on_target)

    def on_target(self, future):
        self._in_flight = False
        try:
            response = future.result()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error(f'Target request failed: {exc}')
            return

        if response.source_exhausted:
            self.get_logger().warn('Source exhausted; signalling the machine')
            self._source_empty = True
            self._set_gpio(SOURCE_EMPTY, 1.0)
            return

        if not response.success:
            self.get_logger().error('Target source reported no valid target')
            return

        p = response.target.point
        command = Float64MultiArray()
        # ---- the boundary. SI stops here, machine units begin. ----
        command.data = xyz_m_to_mm(p.x, p.y, p.z)
        self._cmd_pub.publish(command)
        self._set_gpio(TARGET_READY, 1.0)

        self.get_logger().info(
            f'Target {p.x:.3f}, {p.y:.3f}, {p.z:.3f} m '
            f'in {response.target.header.frame_id} -> commanded')


def main(args=None):
    rclpy.init(args=args)
    node = PlcBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy already shut the context down
    finally:
        rclpy.try_shutdown()
