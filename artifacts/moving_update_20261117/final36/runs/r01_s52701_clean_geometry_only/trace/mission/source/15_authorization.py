"""Deterministic execution-authorization helpers (no ROS dependency)."""
import hashlib
import math


def _f(value):
    return f"{float(value):.9f}"


def _stamp(stamp):
    if stamp is None:
        return "0.0"
    return (f"{int(getattr(stamp, 'sec', 0))}."
            f"{int(getattr(stamp, 'nanosec', 0))}")


def path_digest(path):
    """Hash all execution-relevant fields of a nav_msgs/Path-like object.

    The function intentionally accepts duck-typed test objects so gate and
    RuleGuard can share deterministic offline vectors without importing ROS.
    """
    header = getattr(path, "header", None)
    rows = [
        str(getattr(header, "frame_id", "")),
        _stamp(getattr(header, "stamp", None)),
    ]
    for stamped in list(getattr(path, "poses", []) or []):
        pose = getattr(stamped, "pose", None)
        pos = getattr(pose, "position", None)
        ori = getattr(pose, "orientation", None)
        pheader = getattr(stamped, "header", None)
        rows.append("|".join([
            str(getattr(pheader, "frame_id", "")),
            _stamp(getattr(pheader, "stamp", None)),
            _f(getattr(pos, "x", 0.0)),
            _f(getattr(pos, "y", 0.0)),
            _f(getattr(pos, "z", 0.0)),
            _f(getattr(ori, "x", 0.0)),
            _f(getattr(ori, "y", 0.0)),
            _f(getattr(ori, "z", 0.0)),
            _f(getattr(ori, "w", 1.0)),
        ]))
    return hashlib.sha256("\n".join(rows).encode()).hexdigest()


def token_id(epoch, snapshot_hash, digest):
    """Stable, content-addressed identifier for one admitted execution."""
    raw = f"{int(epoch)}|{str(snapshot_hash)}|{str(digest)}"
    return hashlib.sha256(raw.encode()).hexdigest()[:24]


def verify_authorization(path, claimed_digest, claimed_token, epoch,
                         snapshot_hash):
    """Verify the content-addressed fields of an authorization message.

    Returns ``(ok, reason, actual_digest)``. Identity agreement with the
    independently observed manager state is intentionally a separate check.
    """
    actual = path_digest(path)
    if actual != str(claimed_digest):
        return False, "authorization-path-digest-mismatch", actual
    expected = token_id(epoch, snapshot_hash, actual)
    if expected != str(claimed_token):
        return False, "authorization-token-mismatch", actual
    return True, "ok", actual


def connector_is_safe(start_distance, current_clearance, tolerance):
    """Conservative permission for actual-pose -> first-waypoint connector.

    Signed distance is 1-Lipschitz.  If the connector length is strictly less
    than the current clearance from the already inflated forbidden set, every
    point on that connector remains outside the set.  ``tolerance`` also
    prevents delayed/replayed paths from starting far from the actual robot.
    """
    try:
        distance = float(start_distance)
        clearance = float(current_clearance)
        limit = float(tolerance)
    except Exception:
        return False
    if (not math.isfinite(distance) or distance < 0.0
            or not math.isfinite(limit) or limit < 0.0
            or math.isnan(clearance) or clearance == -math.inf):
        return False
    if distance > limit:
        return False
    return clearance == math.inf or distance < clearance
