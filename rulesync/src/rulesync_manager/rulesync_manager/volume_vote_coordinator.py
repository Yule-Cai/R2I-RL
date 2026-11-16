"""Opt-in isolated volume vote transport, reusing the existing spinner.

Certificate-bound BridgeVoteVolume requests are unchanged. No nested manager
spins and no helper retry after an uncertain request. Helper remains default.
"""
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from rulesync_interfaces.msg import RuleVolume
from rulesync_interfaces.srv import BridgeVoteVolume
from .vote_coordinator import VoteCoordinator, TIMEOUT


def build_volume_request(volumes, epoch, op, txn_id="", target_hash=""):
    request = BridgeVoteVolume.Request()
    request.epoch, request.op = int(epoch), str(op)
    request.txn_id, request.target_hash = str(txn_id), str(target_hash)
    result = []
    for data in volumes:
        volume = RuleVolume()
        volume.rule_id, volume.epoch = str(data["rule_id"]), int(data["epoch"])
        volume.rule_type = int(data["rule_type"])
        volume.severity = int(data.get("severity", 1))
        volume.frame_id = str(data.get("frame_id", "map"))
        volume.geometry_type = int(data["geometry_type"])
        volume.center.x, volume.center.y, volume.center.z = map(float, data["center"])
        volume.dimensions.x, volume.dimensions.y, volume.dimensions.z = map(float, data["dims"])
        volume.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                           for p in data.get("vertices", [])]
        vf, vu = data["valid_from"], data["valid_until"]
        volume.valid_from = Time(sec=int(vf[0]), nanosec=int(vf[1]))
        volume.valid_until = Time(sec=int(vu[0]), nanosec=int(vu[1]))
        result.append(volume)
    request.volume_rules = result
    return request


class VolumeVoteCoordinator(VoteCoordinator):
    def __init__(self, info=None):
        super().__init__(["uav_bridge"], info=info,
            request_builder=build_volume_request, service_type=BridgeVoteVolume,
            service_name=lambda p: f"/rulesync/{p}_vote_volume",
            node_name="rulesync_volume_vote_coordinator")

    def round(self, *args, **kwargs):
        votes = super().round(*args, **kwargs)
        return {part: reply if reply[0] in (0, 1, 2)
                else (TIMEOUT, "invalid-volume-vote", reply[2])
                for part, reply in votes.items()}
