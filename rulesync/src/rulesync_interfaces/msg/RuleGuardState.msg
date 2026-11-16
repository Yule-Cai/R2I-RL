# Runtime state published by the independent RuleGuard process.

std_msgs/Header header

uint8 CLEAR=0
uint8 WARNING=1
uint8 HOLD=2
uint8 state

bool degraded
string reason
float64 clearance

uint64 authority_epoch
string authority_hash
uint64 execution_epoch
string execution_hash
string token_id
