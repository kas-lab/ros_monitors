from ros_monitors_functions.utils import register_monitor
import numpy as np



@register_monitor("test_composite")
def test_composite(_, msg, msg_two):
    return "Composed" + str(msg) + str(msg_two)


def _point_seg_dist(p, a, b):
    """
    Distance from point(s) p to segment(s) (a, b).
    p, a, b are each either shape (2,) or (N, 2); numpy broadcasting
    handles "one point vs many segments" and "many points vs one segment"
    with the same code.
    """
    p = np.atleast_2d(p)
    a = np.atleast_2d(a)
    b = np.atleast_2d(b)

    ab = b - a
    len_sq = np.einsum("ij,ij->i", ab, ab)
    len_sq_safe = np.where(len_sq < 1e-12, 1.0, len_sq)
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / len_sq_safe, 0.0, 1.0)
    proj = a + ab * t[:, None]
    return np.linalg.norm(p - proj, axis=1)


def person_heading_intersects_path(path_poses, person, buffer=5.0):
    """
    path_poses: list of geometry_msgs/PoseStamped (or anything with
                .pose.position.x / .y)
    person:     a people_msgs/Person (or anything with .position.x/.y,
                .velocity.x/.y, .reliability)
    """

    vel = np.array([person.velocity.x, person.velocity.y])
    speed = np.linalg.norm(vel)
    if speed < 0.05:
        return False  # no reliable heading to extrapolate

    pos = np.array([person.position.x, person.position.y])
    future = pos + (vel / speed) * 5.0  # proj distance

    path_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in path_poses])
    a, b = path_xy[:-1], path_xy[1:]  # each shape (M, 2): path segment endpoints

    # Exact intersection test, vectorized across all M path segments at once.
    r, s = future - pos, b - a
    rxs = r[0] * s[:, 1] - r[1] * s[:, 0]
    qp = a - pos
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]) / rxs
        u = (qp[:, 0] * r[1] - qp[:, 1] * r[0]) / rxs
    valid = np.abs(rxs) > 1e-9
    if np.any(valid & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)):
        return True

    if buffer > 0.0:
        # Closest approach between the person's projected segment and each
        # path segment: min of the four point-to-segment distances per pair.
        d = np.minimum.reduce([
            _point_seg_dist(pos, a, b),
            _point_seg_dist(future, a, b),
            _point_seg_dist(a, pos, future),
            _point_seg_dist(b, pos, future),
        ])
        if np.any(d <= buffer):
            return True

    return False


@register_monitor("any_person_heading_intersects_path")
def any_person_heading_intersects_path(path_poses, people):
    """people: a people_msgs/People message (or any object with .people list)."""
    return any(
        person_heading_intersects_path(path_poses, person)
        for person in people.people
    )
