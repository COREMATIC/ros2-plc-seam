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
4. On the next GPIO cycle we raise ``target_ready`` with ``target_id``, the
   PLC's number for the request we answer. The PLC moves only if the number
   matches, and drops ``request_target``.
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
    EMPTY_ACK, GPIO_COMMANDS, GPIO_STATES, REQUEST_ID, REQUEST_TARGET, SETPOINT_COMMANDS,
    SOURCE_EMPTY, TARGET_ID, TARGET_READY)
from plc_seam.units import xyz_m_to_mm


class PlcBridge(Node):

    def __init__(self):
        super().__init__('plc_bridge')

        # Targets in any other frame are refused: the PLC reads raw numbers.
        self.declare_parameter('base_frame', 'base_link')
        # A call with no answer after this long is dropped, so the next
        # request edge is served again (a source that died mid-call).
        self.declare_parameter('service_timeout_s', 5.0)

        self._prev_request = 0.0
        self._request_id = None             # the PLC's number for the open request
        self._answer_id = None              # setpoint sent; target_ready goes out next cycle
        self._in_flight = None              # (future, start time) of the pending call
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
        values = msg.interface_values[i].values
        return values[0] if values else None

    def _set_gpio(self, name: str, value: float):
        self._set_gpios({name: value})

    def _set_gpios(self, values: dict):
        """One message: the controller writes all of them in the same cycle."""
        msg = DynamicInterfaceGroupValues()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = list(values)
        msg.interface_values = [
            InterfaceValue(interface_names=['command'], values=[float(v)])
            for v in values.values()]
        self._gpio_pub.publish(msg)

    # ---- the cycle -------------------------------------------------------

    def on_gpio_states(self, msg: DynamicInterfaceGroupValues):
        # Booleans cross the ros2_control boundary as float64. Comparing
        # against 1.0 looks wrong and is correct.
        request = self._read_gpio(msg, REQUEST_TARGET)
        if request is None:
            return

        request_id = self._read_gpio(msg, REQUEST_ID)
        self._expire_stale_call()
        if self._answer_id is not None:
            # The setpoint went out on the previous cycle; now the flag, with the
            # number of the request it answers. The PLC checks that number, so a
            # flag that overtakes its setpoint, or answers an older request, is
            # ignored instead of moving the machine to the previous target.
            if request == 1.0 and request_id == self._answer_id:
                self._set_gpios({TARGET_ID: self._answer_id, TARGET_READY: 1.0})
            else:
                self.get_logger().warn('Request changed before the answer went out; discarded')
            self._answer_id = None
        if request == 1.0 and self._prev_request != 1.0 and self._in_flight is None:
            self._request_id = request_id
            self.request_target()
        elif request != 1.0 and (
                self._prev_request == 1.0 or self._read_gpio(msg, TARGET_READY) == 1.0):
            # The machine finished the move; drop our ready flag. The read-back
            # also clears a flag left high by an earlier run of this node.
            self._set_gpio(TARGET_READY, 0.0)

        self._prev_request = request

        if self._read_gpio(msg, EMPTY_ACK) == 1.0 and (
                self._source_empty or self._read_gpio(msg, SOURCE_EMPTY) == 1.0):
            self._source_empty = False
            self._set_gpio(SOURCE_EMPTY, 0.0)

    def _expire_stale_call(self):
        if self._in_flight is None:
            return
        future, start = self._in_flight
        timeout_s = self.get_parameter('service_timeout_s').value
        if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout_s:
            self.get_logger().error('Target source did not answer in time; dropping the call')
            self._in_flight = None
            self._client.remove_pending_request(future)
            future.cancel()

    def request_target(self):
        if not self._client.service_is_ready():
            self.get_logger().warn('No target source available; skipping cycle')
            return
        future = self._client.call_async(GetTarget.Request())
        self._in_flight = (future, self.get_clock().now())
        future.add_done_callback(self.on_target)

    def on_target(self, future):
        if self._in_flight is None or self._in_flight[0] is not future:
            return                          # dropped by _expire_stale_call
        self._in_flight = None
        if future.cancelled():
            return
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

        if self._prev_request != 1.0:
            # The PLC gave up on this request (its watchdog) before we answered.
            # A late target_ready would sit high while the PLC waits for it to drop.
            self.get_logger().warn('Target arrived after the PLC dropped its request; discarded')
            return

        base_frame = self.get_parameter('base_frame').value
        if response.target.header.frame_id != base_frame:
            self.get_logger().error(
                f'Target in frame {response.target.header.frame_id!r}, '
                f'not {base_frame!r}: not commanded')
            return

        p = response.target.point
        command = Float64MultiArray()
        # ---- the boundary. SI stops here, machine units begin. ----
        command.data = xyz_m_to_mm(p.x, p.y, p.z)
        self._cmd_pub.publish(command)
        # target_ready and target_id go out on the next GPIO cycle, after the
        # setpoint, as one message (see on_gpio_states).
        self._answer_id = self._request_id

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
