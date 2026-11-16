# Valid durable target, not an authenticated credential or release permit.
# available=false means authority is unknown: fail closed.
# ready=true additionally requires exact participant convergence.
bool available
uint64 epoch
string snapshot_hash
bool ready
rulesync_interfaces/RuleVolume[] volumes
