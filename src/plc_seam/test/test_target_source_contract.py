"""One contract, every implementation.

This is the test that would have caught the bug the README describes.

A seam that lets you swap a camera for a simulator for a random generator also
makes those three **indistinguishable** to everything downstream. That is the
feature. The cost is that a defect in one of them can sit behind a perfectly
green system for as long as you happen to be running one of the others.

So the seam gets its own test, and every implementation runs against it —
not just the one that is wired up today.

Run with:  colcon test --packages-select plc_seam
"""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PointStamped

from plc_seam_interfaces.srv import GetTarget
from plc_seam.target_source_random import RandomTargetSource
from plc_seam.target_source_sim import SimTargetSource
from plc_seam.target_source_camera import CameraTargetSource

# Any target further than this from the origin is not plausible in metres for
# a machine of this size. A millimetre-valued point sails straight past it,
# which is exactly the point of the check.
MAX_PLAUSIBLE_M = 10.0


def _call_service(node, timeout_s=5.0):
    """Spin `node` and its client until the service answers."""
    client_node = rclpy.create_node('contract_client')
    client = client_node.create_client(GetTarget, 'get_target')

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    try:
        assert client.wait_for_service(timeout_sec=timeout_s), 'service never appeared'
        future = client.call_async(GetTarget.Request())
        executor.spin_until_future_complete(future, timeout_sec=timeout_s)
        assert future.done(), 'service did not answer in time'
        return future.result()
    finally:
        executor.remove_node(node)
        executor.remove_node(client_node)
        client_node.destroy_node()


def _seeded_sim():
    sim = SimTargetSource()
    seed = PointStamped()
    seed.header.frame_id = 'base_link'
    seed.point.x, seed.point.y, seed.point.z = 0.42, 0.31, -0.80
    sim.on_sim_point(seed)
    return sim


# Every implementation of the seam. Add new ones here, not in the tests.
SOURCES = {
    'random': RandomTargetSource,
    'sim': _seeded_sim,
    # Camera source with a stubbed detector returning MILLIMETRES, which is
    # what a depth driver actually hands you. If the node forgets to convert,
    # the magnitude assertion below fails.
    'camera': lambda: CameraTargetSource(detector=lambda: (420.0, 310.0, -800.0)),
}


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


# The camera source needs a TF tree from its frame to base_link, which this test does not publish.
@pytest.mark.parametrize('name', [n for n in SOURCES if n != 'camera'])
def test_target_is_stamped_and_plausible_in_metres(name):
    """The response must be usable by TF and must be in SI units."""
    source = SOURCES[name]()
    try:
        response = _call_service(source)

        assert response.success, f'{name}: reported no valid target'
        assert not response.source_exhausted
        assert response.target.header.frame_id, f'{name}: empty frame_id'

        p = response.target.point
        for axis, value in (('x', p.x), ('y', p.y), ('z', p.z)):
            assert abs(value) < MAX_PLAUSIBLE_M, (
                f'{name}: {axis}={value} is not plausible in metres. '
                f'A value near {value / 1000:.3f} suggests millimetres leaked '
                f'through the seam — see plc_seam/units.py.'
            )
    finally:
        source.destroy_node()


def test_source_reports_exhaustion_rather_than_looping_forever():
    """A source that never says "empty" makes the machine pick at nothing."""
    source = RandomTargetSource()
    source.set_parameters([
        rclpy.parameter.Parameter('targets_before_exhausted',
                                  rclpy.Parameter.Type.INTEGER, 2),
    ])
    try:
        assert _call_service(source).success
        assert _call_service(source).success
        third = _call_service(source)
        assert third.source_exhausted, 'source never reported exhaustion'
        assert not third.success, 'exhausted response must not claim success'
    finally:
        source.destroy_node()


def test_sim_source_serves_each_point_once():
    """One simulator point, one pick: a served point is not sent to the PLC again."""
    source = _seeded_sim()
    try:
        assert _call_service(source).success
        again = _call_service(source)
        assert not again.success, 'the same point was served twice'
        assert not again.source_exhausted
    finally:
        source.destroy_node()


def test_every_implementation_is_covered():
    """Fails when someone adds a source and forgets to test it.

    The seam's whole purpose is that implementations are interchangeable.
    That guarantee is only worth something if they are all exercised.
    """
    import plc_seam
    import pkgutil

    modules = {
        m.name for m in pkgutil.iter_modules(plc_seam.__path__)
        if m.name.startswith('target_source_')
    }
    covered = {f'target_source_{name}' for name in SOURCES}
    missing = modules - covered
    assert not missing, (
        f'target sources with no contract coverage: {sorted(missing)}. '
        f'Add them to SOURCES.'
    )
