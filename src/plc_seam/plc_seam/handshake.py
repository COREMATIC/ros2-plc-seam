"""The names both sides of the handshake agree on, in one place.

The GPIO names are the ``<gpio>`` names in ``urdf/plc_ros2_control.xacro``; the
topics are those of the two controllers in ``config/controllers.yaml``.
``plc_bridge_node.py`` and ``plc_sim.py`` both import them from here.
"""

# Written by the PLC.
REQUEST_TARGET = 'request_target'
EMPTY_ACK = 'empty_ack'
PICKING = 'picking'
PLACING = 'placing'
# Written by ROS.
TARGET_READY = 'target_ready'
SOURCE_EMPTY = 'source_empty'

GPIOS = (REQUEST_TARGET, TARGET_READY, SOURCE_EMPTY, EMPTY_ACK, PICKING, PLACING)
WRITTEN_BY_ROS = (TARGET_READY, SOURCE_EMPTY)

GPIO_COMMANDS = '/gpio_command/commands'
GPIO_STATES = '/gpio_command/gpio_states'
SETPOINT_COMMANDS = '/forward_command/commands'
