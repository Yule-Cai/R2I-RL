# Deterministic structured runtime rule (trusted interface).
# No natural language. All geometry in frame_id, finite, validated.

string rule_id          # unique id; re-propose with higher epoch = versioned update

uint64 epoch            # caller-proposed epoch; must exceed committed epoch

uint8 rule_type         # 0=KEEP_OUT, 1=SPEED_LIMIT
uint8 KEEP_OUT=0
uint8 SPEED_LIMIT=1

uint8 severity          # 0=advisory, 1=enforced (hard)
uint8 ADVISORY=0
uint8 ENFORCED=1

string frame_id         # must be an allowed frame (default: map)

geometry_msgs/Polygon polygon   # >=3 finite points, closed implicitly

float32 speed_limit     # m/s, >0, only meaningful for SPEED_LIMIT

builtin_interfaces/Time valid_from
builtin_interfaces/Time valid_until   # must be > valid_from
