# Immutable output of the geometric admission gate.  The complete candidate
# path is carried in the same ROS message as the committed snapshot identity,
# avoiding cross-topic token/path pairing ambiguity.

std_msgs/Header header
string token_id
uint64 epoch
string snapshot_hash
string path_digest
string gate_reason
nav_msgs/Path path
