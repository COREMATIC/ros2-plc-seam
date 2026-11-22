"""Unit policy for this workspace.

The rule
--------
Everything on the ROS side of this system is **SI (metres)**, per REP-103.
The machine speaks **millimetres**, because PLCs generally do.

Convert **exactly once**, at the boundary with the machine, using the helpers
below. Never carry millimetres into a message that has a frame_id, and never
hand a millimetre-valued point to TF.

Why this file exists
--------------------
In the system this reference is drawn from, the millimetre convention leaked
upward from the PLC because it was convenient at the edge. It reached a point
where a millimetre-valued ``PointStamped`` was passed to
``tf2_geometry_msgs.do_transform_point`` together with a transform whose
translation was in metres.

The rotation was applied correctly. The translation was added in the wrong
unit, so it contributed a thousand times less than intended, and the detected
target effectively stopped moving the machine.

Nothing caught it: ROS message types carry no unit, no joint-limit enforcement
was configured on that path, and every value still looked plausible.

A named conversion at a single boundary is the cheap fix. The contract test in
``test/test_target_source_contract.py`` is the cheap detector.
"""

MM_PER_M = 1000.0


def mm_to_m(value_mm: float) -> float:
    """Convert a scalar from millimetres to metres."""
    return value_mm / MM_PER_M


def m_to_mm(value_m: float) -> float:
    """Convert a scalar from metres to millimetres.

    Call this at the machine boundary and nowhere else.
    """
    return value_m * MM_PER_M


def point_mm_to_m(point) -> None:
    """Convert a ``geometry_msgs/Point`` in place, millimetres to metres.

    Use this the moment a millimetre-valued reading enters the ROS side —
    before it is stamped with a frame, and long before it reaches TF.
    """
    point.x = mm_to_m(point.x)
    point.y = mm_to_m(point.y)
    point.z = mm_to_m(point.z)


def xyz_m_to_mm(x_m: float, y_m: float, z_m: float):
    """Convert an (x, y, z) triple to millimetres for the machine."""
    return [m_to_mm(x_m), m_to_mm(y_m), m_to_mm(z_m)]
