# Staged rule set broadcast for 2-phase commit (manager -> bridges).
# Transient-local latched; bridges vote on per-participant vote topics.
# txn_id + target_hash bind the prepared identity (correctness revision).
rulesync_interfaces/RuleSpec[] rules
uint64 epoch
string op
string txn_id
string target_hash
