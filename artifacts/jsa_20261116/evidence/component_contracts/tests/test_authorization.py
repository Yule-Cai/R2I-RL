import types

from rulesync3d.rulesync3d.authorization import (
    connector_is_safe, path_digest, token_id, verify_authorization,
)


def _path(x2=2.0, frame="map", stamp=(7, 11)):
    header = types.SimpleNamespace(
        frame_id=frame,
        stamp=types.SimpleNamespace(sec=stamp[0], nanosec=stamp[1]))

    def pose(x, y):
        return types.SimpleNamespace(
            header=header,
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y, z=0.0),
                orientation=types.SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0)))

    return types.SimpleNamespace(header=header,
                                 poses=[pose(0.0, 0.0), pose(x2, 1.0)])


def test_path_digest_is_deterministic_and_geometry_complete():
    assert path_digest(_path()) == path_digest(_path())
    assert path_digest(_path()) != path_digest(_path(x2=2.001))
    assert path_digest(_path()) != path_digest(_path(frame="odom"))
    assert path_digest(_path()) != path_digest(_path(stamp=(8, 11)))


def test_token_binds_epoch_snapshot_and_path():
    digest = path_digest(_path())
    base = token_id(7, "abc", digest)
    assert base == token_id(7, "abc", digest)
    assert base != token_id(8, "abc", digest)
    assert base != token_id(7, "def", digest)
    assert base != token_id(7, "abc", path_digest(_path(x2=3.0)))


def test_authorization_verifier_rejects_digest_and_token_mutation():
    path = _path()
    digest = path_digest(path)
    token = token_id(7, "abc", digest)
    assert verify_authorization(path, digest, token, 7, "abc")[:2] \
        == (True, "ok")
    assert verify_authorization(path, "bad", token, 7, "abc")[:2] \
        == (False, "authorization-path-digest-mismatch")
    assert verify_authorization(path, digest, "bad", 7, "abc")[:2] \
        == (False, "authorization-token-mismatch")


def test_connector_requires_local_start_and_clearance_budget():
    assert connector_is_safe(0.10, 0.30, 0.25)
    assert not connector_is_safe(0.30, 1.00, 0.25)  # stale/far start
    assert not connector_is_safe(0.20, 0.20, 0.25)  # touches boundary
    assert not connector_is_safe(0.20, 0.10, 0.25)  # crosses safe ball
    assert connector_is_safe(0.20, float("inf"), 0.25)  # no keepouts
    assert not connector_is_safe(0.20, float("nan"), 0.25)
