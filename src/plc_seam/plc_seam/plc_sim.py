"""A stand-in for the PLC, so the whole cycle runs with no machine.

The bridge never talks to the PLC directly. It talks to two controllers:
``/gpio_command`` for the handshake and ``/forward_command`` for setpoints.
Publish the same topics from a plain node and the bridge cannot tell the
difference. That is what this node does:

1. It raises ``request_target`` and waits for ``target_ready`` together with
   a setpoint received since the request.
2. It checks the setpoint against its working window. Outside the window it
   refuses, drops the request and asks again: the PLC has the last word.
3. Inside the window it drops the request (the bridge then clears
   ``target_ready``), moves there at a fixed speed, places, and asks again.
4. When the bridge raises ``source_empty``, it waits for a refill and
   acknowledges with ``empty_ack``.
5. When no valid answer comes within ``request_timeout_s`` (the source had
   nothing this cycle, or nobody serves ``get_target``), it drops the request
   and asks again. The watchdog lives on the PLC side: the machine waits, it
   never moves on a guess.

Setpoints arrive in millimetres, as the machine would read them; joint states
leave in metres, as ``joint_state_broadcaster`` would publish them.

It replaces ``ros2_control_node`` and its three controllers in
``bringup.launch.py plc:=sim``. Nothing else changes.

Subscribes: /forward_command/commands  (std_msgs/Float64MultiArray, MILLIMETRES)
            /gpio_command/commands     (control_msgs/DynamicInterfaceGroupValues)
Publishes:  /gpio_command/gpio_states  (control_msgs/DynamicInterfaceGroupValues)
            /joint_states              (sensor_msgs/JointState, METRES)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from plc_seam.handshake import (
    GPIO_COMMANDS, GPIO_STATES, GPIOS, SETPOINT_COMMANDS, WRITTEN_BY_ROS)
from plc_seam import envelope
from plc_seam.units import m_to_mm, mm_to_m

JOINTS = ('x_joint', 'y_joint', 'z_joint',
          'transfer_joint', 'gripper_left_joint', 'gripper_right_joint')


class PlcModel:
    """The PLC side of the handshake as a plain state machine, without ROS.

    ``step(dt)`` advances it; ``on_command`` and ``on_setpoint`` feed it what
    the bridge writes. Positions are millimetres, like the machine's.
    """

    def __init__(self, window_mm, home_mm=(0.0, 0.0, -400.0),
                 place_mm=(0.0, 0.0, -400.0), speed_mm_s=500.0,
                 place_s=1.0, refill_s=3.0, retry_s=0.5, request_timeout_s=5.0):
        self.window_mm = window_mm          # ((xmin, xmax), (ymin, ymax), (zmin, zmax))
        self.place_mm = list(place_mm)
        self.speed_mm_s = speed_mm_s
        self.place_s, self.refill_s, self.retry_s = place_s, refill_s, retry_s
        self.request_timeout_s = request_timeout_s

        self.pos_mm = list(home_mm)
        self.gpio = {name: 0.0 for name in GPIOS}
        self.holding = False
        self.picks = 0
        self.refused = 0
        self.timeouts = 0

        self._setpoint = None
        self._fresh = False                 # a setpoint arrived since the request rose
        self._goal = None
        self._timer = 0.0
        self._ask()

    # ---- what the bridge writes --------------------------------------------

    def on_command(self, name, value):
        if name in WRITTEN_BY_ROS and not math.isnan(value):
            self.gpio[name] = value         # the PLC reads back what ROS wrote

    def on_setpoint(self, xyz_mm):
        self._setpoint = list(xyz_mm)
        self._fresh = self.state == 'request'

    # ---- the cycle ------------------------------------------------------------

    def _ask(self):
        self.state = 'request'
        self._fresh = False
        self._timer = self.request_timeout_s
        self.gpio['request_target'] = 1.0

    def in_window(self, xyz_mm):
        return all(lo <= v <= hi for v, (lo, hi) in zip(xyz_mm, self.window_mm))

    def _move(self, dt):
        """Move towards the goal; True once there."""
        delta = [g - p for g, p in zip(self._goal, self.pos_mm)]
        dist = math.sqrt(sum(d * d for d in delta))
        reach = self.speed_mm_s * dt
        if dist <= reach:
            self.pos_mm = list(self._goal)
            return True
        self.pos_mm = [p + d * reach / dist for p, d in zip(self.pos_mm, delta)]
        return False

    def step(self, dt):
        g = self.gpio
        self._timer -= dt                   # only the timed states read it
        if self.state == 'request':
            if g['source_empty'] == 1.0:
                g['request_target'] = 0.0
                self.state, self._timer = 'refill', self.refill_s
            elif g['target_ready'] == 1.0 and self._fresh:
                g['request_target'] = 0.0
                if self.in_window(self._setpoint):
                    self._goal = self._setpoint
                    g['picking'] = 1.0
                    self.state = 'pick'
                else:
                    self.refused += 1
                    self.state, self._timer = 'retry', self.retry_s
            elif self._timer <= 0.0:            # the watchdog: ask again
                self.timeouts += 1
                g['request_target'] = 0.0
                self.state, self._timer = 'retry', self.retry_s
        elif self.state == 'retry':
            if self._timer <= 0.0 and g['target_ready'] == 0.0:
                self._ask()
        elif self.state == 'pick':
            if self._move(dt):
                self.holding = True
                g['picking'], g['placing'] = 0.0, 1.0
                self._goal = self.place_mm
                self.state = 'place'
        elif self.state == 'place':
            if self._move(dt):
                self.state, self._timer = 'release', self.place_s
        elif self.state == 'release':
            if self._timer <= 0.0:
                self.holding = False
                g['placing'] = 0.0
                self.picks += 1
                if g['target_ready'] == 0.0:
                    self._ask()
                else:
                    self.state = 'retry'
        elif self.state == 'refill':
            if self._timer <= 0.0:
                g['empty_ack'] = 1.0        # an operator refilled the source
            if g['empty_ack'] == 1.0 and g['source_empty'] == 0.0:
                g['empty_ack'] = 0.0
                self._ask()

    def joint_positions_m(self):
        """Joint values in the order of JOINTS, in metres."""
        grip = 0.0 if self.holding else 0.04
        transfer = 0.3 if self.state in ('place', 'release') else 0.0
        return [mm_to_m(v) for v in self.pos_mm] + [transfer, grip, grip]


class PlcSim(Node):

    def __init__(self):
        super().__init__('plc_sim')
        # The PLC's own working window, in METRES like every ROS parameter;
        # config/envelope.yaml by default. A setpoint outside it is refused,
        # whatever the target source said.
        window = envelope.load()
        for axis in envelope.AXES:
            self.declare_parameter(f'{axis}_window_m', list(window[axis]))
        self.declare_parameter('speed_mm_s', 500.0)
        self.declare_parameter('refill_s', 3.0)
        self.declare_parameter('request_timeout_s', 5.0)
        self.declare_parameter('rate_hz', 40.0)

        def p(name):
            return self.get_parameter(name).value

        self.model = PlcModel(
            # The PLC's limits are in its units: convert once, here.
            window_mm=tuple(tuple(m_to_mm(v) for v in p(f'{axis}_window_m'))
                            for axis in envelope.AXES),
            speed_mm_s=p('speed_mm_s'), refill_s=p('refill_s'),
            request_timeout_s=p('request_timeout_s'))
        self._dt = 1.0 / p('rate_hz')

        self._gpio_pub = self.create_publisher(
            DynamicInterfaceGroupValues, GPIO_STATES, 10)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(
            DynamicInterfaceGroupValues, GPIO_COMMANDS, self.on_gpio, 10)
        self.create_subscription(
            Float64MultiArray, SETPOINT_COMMANDS, self.on_setpoint, 10)
        self.create_timer(self._dt, self.tick)
        self._logged = {'picks': 0, 'refused': 0, 'timeouts': 0}
        self.get_logger().info('PLC stand-in running: no machine connected')

    def on_gpio(self, msg):
        for name, iv in zip(msg.interface_groups, msg.interface_values):
            if 'command' in iv.interface_names:
                self.model.on_command(name, iv.values[iv.interface_names.index('command')])

    def on_setpoint(self, msg):
        if len(msg.data) == 3:
            self.model.on_setpoint(msg.data)

    def _changed(self, counter):
        value = getattr(self.model, counter)
        changed, self._logged[counter] = value != self._logged[counter], value
        return changed

    def tick(self):
        self.model.step(self._dt)
        m = self.model
        if self._changed('refused'):
            self.get_logger().warn('Setpoint outside the working window: refused')
        if self._changed('timeouts'):
            self.get_logger().warn('No valid target in time: asking again')
        if self._changed('picks'):
            self.get_logger().info(f'Pick {m.picks} done')

        now = self.get_clock().now().to_msg()
        states = DynamicInterfaceGroupValues()
        states.header.stamp = now
        states.interface_groups = list(GPIOS)
        states.interface_values = [
            InterfaceValue(interface_names=['state'], values=[m.gpio[n]]) for n in GPIOS]
        self._gpio_pub.publish(states)

        joints = JointState()
        joints.header.stamp = now
        joints.name = list(JOINTS)
        joints.position = m.joint_positions_m()
        self._joint_pub.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    node = PlcSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy already shut the context down
    finally:
        rclpy.try_shutdown()
