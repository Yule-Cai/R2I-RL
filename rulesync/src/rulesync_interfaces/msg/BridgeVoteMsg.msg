# Bridge vote broadcast (bridge -> manager), transient-local latched.
string participant
uint8 vote
uint8 READY=0
uint8 REJECT=1
uint8 TIMEOUT=2
string reason
uint64 epoch
string op
