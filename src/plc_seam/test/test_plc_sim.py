"""The PLC stand-in plays the handshake the bridge expects.

Pure state-machine tests: no ROS graph. The bridge's side is played by hand,
the way plc_bridge_node.py plays it.
"""

from plc_seam import envelope
from plc_seam.plc_sim import PlcModel
from plc_seam.units import m_to_mm

WINDOW = tuple(tuple(m_to_mm(v) for v in envelope.load()[axis]) for axis in envelope.AXES)
DT = 0.025


def run(model, seconds, bridge=None):
    for _ in range(int(seconds / DT)):
        model.step(DT)
        if bridge:
            bridge(model)


def answer(model, xyz_mm, request_id=None):
    """What the bridge does on a rising request_target: setpoint, then target_ready
    with the number of the request it answers (the current one by default)."""
    model.on_setpoint(xyz_mm)
    model.on_command('target_id', model.gpio['request_id'] if request_id is None else request_id)
    model.on_command('target_ready', 1.0)


def drop_ready_on_falling_request(model):
    """The bridge clears target_ready once the PLC drops its request."""
    if model.gpio['request_target'] == 0.0:
        model.on_command('target_ready', 0.0)


def test_one_full_cycle_ends_with_a_new_request():
    m = PlcModel(WINDOW)
    assert m.gpio['request_target'] == 1.0

    answer(m, (400.0, 300.0, -800.0))
    m.step(DT)
    assert m.gpio['request_target'] == 0.0 and m.gpio['picking'] == 1.0

    run(m, 10.0, drop_ready_on_falling_request)
    assert m.picks == 1
    assert m.gpio['request_target'] == 1.0, 'the PLC must ask for the next target'
    assert m.gpio['picking'] == m.gpio['placing'] == 0.0


def test_target_ready_without_a_new_setpoint_is_ignored():
    m = PlcModel(WINDOW)
    m.on_command('target_ready', 1.0)       # a stale flag, no setpoint since the request
    run(m, 1.0)
    assert m.state == 'request' and m.gpio['picking'] == 0.0


def test_answer_to_an_older_request_is_ignored():
    m = PlcModel(WINDOW, request_timeout_s=1.0)
    old = m.gpio['request_id']
    run(m, 1.6)                             # no answer: the watchdog asks again
    assert m.gpio['request_id'] == old + 1 and m.gpio['request_target'] == 1.0

    answer(m, (400.0, 300.0, -800.0), request_id=old)   # a late answer to the first request
    m.step(DT)
    assert m.gpio['picking'] == 0.0, 'a target for an older request must not move the axes'
    assert m.pos_mm == [0.0, 0.0, -400.0]


def test_setpoint_outside_the_window_is_refused_and_the_plc_asks_again():
    m = PlcModel(WINDOW)
    answer(m, (400.0, 300.0, -1500.0))      # below the reachable floor
    m.step(DT)
    assert m.refused == 1 and m.gpio['picking'] == 0.0
    assert m.pos_mm == [0.0, 0.0, -400.0], 'a refused setpoint must not move the axes'

    run(m, 1.0, drop_ready_on_falling_request)
    assert m.gpio['request_target'] == 1.0


def test_empty_source_is_acknowledged_after_the_refill():
    m = PlcModel(WINDOW, refill_s=1.0)
    m.on_command('source_empty', 1.0)
    m.step(DT)
    assert m.gpio['request_target'] == 0.0 and m.state == 'refill'

    run(m, 1.5)
    assert m.gpio['empty_ack'] == 1.0

    m.on_command('source_empty', 0.0)       # the bridge clears it on the ack
    m.step(DT)
    assert m.gpio['empty_ack'] == 0.0 and m.gpio['request_target'] == 1.0


def test_nan_commands_are_ignored():
    m = PlcModel(WINDOW)
    m.on_command('target_ready', float('nan'))
    assert m.gpio['target_ready'] == 0.0


def test_no_answer_in_time_makes_the_plc_ask_again():
    m = PlcModel(WINDOW, request_timeout_s=1.0)
    run(m, 1.1)                             # nobody answers
    assert m.timeouts == 1 and m.gpio['request_target'] == 0.0
    assert m.pos_mm == [0.0, 0.0, -400.0], 'no answer must not move the axes'

    run(m, 1.0)
    assert m.gpio['request_target'] == 1.0, 'a new rising edge for the bridge'
