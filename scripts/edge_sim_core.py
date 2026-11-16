"""Small ROS-free kinematic plant/controller helpers; not physical dynamics."""
import math


def step_unicycle(pose, velocity, dt):
    x, y, theta = pose
    v, omega = velocity
    if dt < 0 or not all(math.isfinite(n) for n in (*pose, *velocity, dt)):
        raise ValueError('finite pose, velocity and nonnegative dt required')
    if abs(omega) < 1e-9:
        x += v * dt * math.cos(theta)
        y += v * dt * math.sin(theta)
    else:
        x += v / omega * (math.sin(theta + omega * dt) - math.sin(theta))
        y -= v / omega * (math.cos(theta + omega * dt) - math.cos(theta))
    return (x, y, (theta + omega * dt + math.pi) % (2 * math.pi) - math.pi)


def track_waypoints(pose, path, index, speed=.35, tolerance=.06):
    """Rotate/advance waypoint follower, deliberately no planner/safety logic."""
    x, y, theta = pose
    while index < len(path) and math.hypot(path[index][0] - x, path[index][1] - y) <= tolerance:
        index += 1
    if index == len(path):
        return (0., 0.), index
    tx, ty = path[index]
    error = (math.atan2(ty - y, tx - x) - theta + math.pi) % (2 * math.pi) - math.pi
    omega = max(-1.2, min(1.2, 3. * error))
    v = min(speed, math.hypot(tx - x, ty - y)) if abs(error) < .25 else 0.
    return (v, omega), index


def feedback_fresh(now, received, timeout):
    return received is not None and 0. <= now - received <= timeout


def release_matches(authority, execution, state_token, expected_token):
    """Cross-topic join: fresh non-HOLD state must bind this exact pending path."""
    return bool(expected_token) and authority == execution and state_token == expected_token


def pending_release_matches(now, received, authority, execution, state_token, expected_token):
    """An expired pending path cannot activate even if its matching state arrives."""
    return feedback_fresh(now, received, 2.) and release_matches(
        authority, execution, state_token, expected_token)
